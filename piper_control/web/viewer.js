import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import URDFLoader from './vendor/URDFLoader.js';

/** URDF and feedback share the same right-handed, Z-up base frame. */
export class RobotViewer {
  constructor(canvas, container, onError) {
    this.container = container;
    this.onError = onError;
    this.robot = null;
    this.state = null;
    this.angleOffsets = [];
    this.trailPoints = [];
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color('#f0f4f8');
    this.scene.fog = new THREE.Fog('#f0f4f8', 3, 8);
    this.camera = new THREE.PerspectiveCamera(38, 1, 0.01, 20);
    this.camera.up.set(0, 0, 1);
    this.renderer = new THREE.WebGLRenderer({canvas, antialias: true, alpha: false});
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 0.95;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.minDistance = 0.25;
    this.controls.maxDistance = 4;
    this.controls.maxPolarAngle = Math.PI * 0.92;
    this.setView('iso');

    this.scene.add(new THREE.HemisphereLight('#ffffff', '#b0bfc9', 1.8));
    const key = new THREE.DirectionalLight('#ffffff', 2.5);
    key.position.set(1, -1.5, 2.4);
    key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048);
    key.shadow.camera.left = -1;
    key.shadow.camera.right = 1;
    key.shadow.camera.top = 1;
    key.shadow.camera.bottom = -1;
    key.shadow.camera.near = 0.1;
    key.shadow.camera.far = 5;
    key.shadow.bias = -0.00015;
    key.shadow.normalBias = 0.002;
    this.scene.add(key);
    const fill = new THREE.DirectionalLight('#cee7ff', 1.2);
    fill.position.set(-1.5, 0.8, 1);
    this.scene.add(fill);

    const floor = new THREE.Mesh(new THREE.PlaneGeometry(200, 200), new THREE.MeshStandardMaterial({color: '#edf2f7', roughness: 1}));
    floor.position.z = -0.005;
    floor.receiveShadow = true;
    this.scene.add(floor);
    const grid = new THREE.GridHelper(3.2, 32, '#c6d4e0', '#dae3eb');
    grid.rotation.x = Math.PI / 2;
    grid.position.z = -0.004;
    grid.material.transparent = true;
    grid.material.opacity = 0.65;
    this.scene.add(grid);
    const baseAxes = new THREE.AxesHelper(0.13);
    baseAxes.position.z = 0.002;
    this.scene.add(baseAxes);
    this.tcpAxes = new THREE.AxesHelper(0.075);
    this.tcpAxes.visible = false;
    this.scene.add(this.tcpAxes);
    this.trail = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({color: '#4b99cf', transparent: true, opacity: 0.75}));
    this.scene.add(this.trail);
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(container);
    this.resize();
    this.renderer.setAnimationLoop(() => {
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
    });
  }

  resize() {
    const {width, height} = this.container.getBoundingClientRect();
    if (!width || !height) return;
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);
  }

  async load(url) {
    const manager = new THREE.LoadingManager();
    const loader = new URDFLoader(manager);
    loader.packages = {piper_description: '/robot'};
    manager.onError = path => this.onError(`模型资源加载失败：${path}`);
    return new Promise((resolve, reject) => {
      let robot;
      manager.onLoad = () => {
        if (!robot) return;
        robot.traverse(object => {
          if (!object.isMesh) return;
          object.castShadow = true;
          object.receiveShadow = true;
          const color = object.material?.color?.clone() || new THREE.Color('#a9bac7');
          // Keep the URDF colors, with consistent PBR illumination for STL meshes.
          object.material = new THREE.MeshStandardMaterial({color, metalness: 0.28, roughness: 0.43});
        });
        this.robot = robot;
        this.scene.add(robot);
        if (this.state) this.update(this.state);
        resolve();
      };
      loader.load(url, result => { robot = result; }, undefined, reject);
    });
  }

  setView(view) {
    const target = new THREE.Vector3(0.12, 0, 0.26);
    const views = {iso: [0.90, -0.85, 0.70], front: [1.40, 0, 0.32], side: [0.12, -1.30, 0.32], top: [0.12, -0.001, 1.5]};
    this.camera.position.fromArray(views[view] || views.iso);
    this.controls.target.copy(target);
    this.controls.update();
  }

  update(state) {
    this.state = state;
    if (this.robot && state.positions) {
      const names = state.joint_names || Object.keys(this.robot.joints).slice(0, state.positions.length);
      names.forEach((name, i) => this.robot.setJointValue(name, state.positions[i] + (this.angleOffsets[i] || 0)));
    }
    const pose = state.tcp_pose;
    if (!pose?.position || !pose?.orientation) return;
    this.tcpAxes.visible = true;
    this.tcpAxes.position.fromArray(pose.position);
    this.tcpAxes.quaternion.fromArray(pose.orientation);
    const p = this.tcpAxes.position.clone();
    const previous = this.trailPoints[this.trailPoints.length - 1];
    if (!previous || p.distanceTo(previous) > 0.0008) {
      this.trailPoints.push(p);
      if (this.trailPoints.length > 2000) this.trailPoints.shift();
      this.trail.geometry.dispose();
      this.trail.geometry = new THREE.BufferGeometry().setFromPoints(this.trailPoints);
    }
  }
}
