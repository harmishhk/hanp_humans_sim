export class MoveHumansViz {
  rosBridgeUrl: string;

  ros: ROSLIB.Ros;
  viewer: ROS3D.Viewer;
  gridClient: ROS3D.OccupancyGridClient;
  tfClient: ROSLIB.TFClient;
  tfClientBase: ROSLIB.TFClient;
  urdfClient: ROS3D.UrdfClient;

  constructor (rosBridgeUrl: string) {
    this.rosBridgeUrl = rosBridgeUrl;
  }

  initialize() {
    // connect to ros
    this.ros = new ROSLIB.Ros({
      url: this.rosBridgeUrl
    });

    // create the main viewer
    this.viewer = new ROS3D.Viewer({
      divID: 'map',
      width: window.innerWidth - 20,
      height: window.innerHeight - 20,
      antialias: true,
      cameraPose : {x: 20, y: 10, z: 25},

    });

    // setup a clietn ot litsen to TFs
    this.tfClient = new ROSLIB.TFClient({
      ros: this.ros,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 30.0,
      fixedFrame: 'map'
    });

    // show the map
    this.gridClient = new ROS3D.OccupancyGridClient({
      ros: this.ros,
      rootObject: this.viewer.scene,
      opacity: 0.7
    });

    // setup path clients
    let humanPaths = new ROS3DNAV.PathArray({
      ros: this.ros,
      topic: '/move_humans_node/TeleportController/plans',
      tfClient: this.tfClient,
      rootObject: this.viewer.scene,
      color: 0x0000ff
    });
  }
}

const rosBridgeUrl = "ws://localhost:9090";
let moveHumansViz = new MoveHumansViz(rosBridgeUrl);
moveHumansViz.initialize();

window.addEventListener('resize', onWindowResize, false);
function onWindowResize() {
  moveHumansViz.viewer.resize(window.innerWidth - 20, window.innerHeight - 20);
}
