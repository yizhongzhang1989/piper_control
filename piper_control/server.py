"""Versioned-schema REST and bidirectional WebSocket transports."""
import asyncio
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Annotated, Literal

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, Field
from starlette.concurrency import run_in_threadpool

Finite = Annotated[float, Field(allow_inf_nan=False)]
Positive = Annotated[float, Field(gt=0, allow_inf_nan=False)]


class Request(BaseModel):
    model_config = ConfigDict(extra="forbid")


class Pose(Request):
    position: Annotated[list[Finite], Field(min_length=3, max_length=3)]
    orientation: Annotated[list[Finite], Field(min_length=4, max_length=4)]


class Joints(Request):
    positions: list[Finite]


class MoveJ(Joints):
    duration: Positive | None = None


class MoveL(Request):
    pose: Pose
    duration: Positive | None = None
    speed: Positive = .03
    angular_speed: Positive = .3


class MoveC(MoveL):
    via: Pose


class ServoPose(Request):
    pose: Pose


class Twist(Request):
    linear: Annotated[list[Finite], Field(min_length=3, max_length=3)]
    angular: Annotated[list[Finite], Field(min_length=3, max_length=3)]
    frame: Literal["base", "tool"] = "base"


class Point(Request):
    positions: list[Finite]
    time_from_start: Annotated[float, Field(ge=0, allow_inf_nan=False)]
    velocities: list[Finite] | None = None


class Trajectory(Request):
    points: Annotated[list[Point], Field(min_length=1, max_length=10000)]
    joint_names: list[str] | None = None


def create_app(controller, ros_bridge=None):
    @asynccontextmanager
    async def lifespan(app):
        controller.start()
        yield
        if ros_bridge:
            ros_bridge.close()
        controller.close()

    app = FastAPI(title="Common Robot Control", version="1.0.0", lifespan=lifespan,
                  description="SI units; quaternion xyzw; pose in base_link; common sim/hardware contract.")
    app.state.controller = controller

    @app.exception_handler(ValueError)
    async def invalid(request, exc):
        return JSONResponse({"detail": str(exc)}, status_code=422)

    @app.exception_handler(RuntimeError)
    async def conflict(request, exc):
        return JSONResponse({"detail": str(exc)}, status_code=409)

    @app.exception_handler(KeyError)
    async def missing(request, exc):
        return JSONResponse({"detail": f"not found: {exc}"}, status_code=404)

    @app.get("/api/info")
    def info():
        return controller.info()

    @app.get("/api/state")
    def state():
        return controller.state()

    @app.post("/api/enable")
    def enable():
        return controller.enable()

    @app.post("/api/disable")
    def disable():
        return controller.disable()

    @app.post("/api/stop")
    def stop():
        return controller.stop()

    @app.post("/api/movej")
    def movej(req: MoveJ):
        return controller.movej(**req.model_dump())

    @app.post("/api/movel")
    def movel(req: MoveL):
        return controller.movel(**req.model_dump())

    @app.post("/api/movec")
    def movec(req: MoveC):
        return controller.movec(**req.model_dump())

    @app.post("/api/trajectory")
    def trajectory(req: Trajectory):
        return controller.trajectory(**req.model_dump())

    @app.get("/api/motions/{motion_id}")
    def motion(motion_id: str):
        return controller.motion(motion_id)

    @app.post("/api/stream/joints")
    def joints(req: Joints):
        return controller.servo_joints(**req.model_dump())

    @app.post("/api/stream/tcp")
    def tcp(req: ServoPose):
        return controller.servo_pose(**req.model_dump())

    @app.post("/api/stream/twist")
    def twist(req: Twist):
        return controller.servo_twist(**req.model_dump())

    @app.websocket("/ws/state")
    async def state_stream(ws: WebSocket):
        await ws.accept()
        try:
            while True:
                await ws.send_json(controller.state())
                await asyncio.sleep(.02)
        except (WebSocketDisconnect, RuntimeError):
            pass

    @app.websocket("/ws/control")
    async def control_stream(ws: WebSocket):
        await ws.accept()
        handlers = {"joints": (Joints, controller.servo_joints),
                    "tcp": (ServoPose, controller.servo_pose),
                    "twist": (Twist, controller.servo_twist)}
        try:
            while True:
                data = await ws.receive_json()
                try:
                    kind = data.pop("type")
                    schema, handler = handlers[kind]
                    result = await run_in_threadpool(handler, **schema.model_validate(data).model_dump())
                    await ws.send_json(result)
                except (ValueError, KeyError, TypeError, RuntimeError) as exc:
                    await ws.send_json({"accepted": False, "error": str(exc)})
        except WebSocketDisconnect:
            pass  # The controller watchdog owns the stream timeout on all transports.

    @app.get("/robot/model.urdf")
    def urdf():
        return FileResponse(controller.model.urdf_path, media_type="application/xml")

    app.mount("/robot", StaticFiles(directory=controller.model.asset_dir), name="robot_assets")
    app.mount("/", StaticFiles(directory=Path(__file__).parent/"web", html=True), name="dashboard")
    return app
