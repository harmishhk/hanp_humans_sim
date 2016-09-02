function init() {
  let moveHumansViz = new MoveHumansViz();
  moveHumansViz.initialize(
    'ws://localhost:9090',
    'viewer',
    'map',
    0.7,
    '/move_humans_node/TeleportController/plans',
    '/move_humans_node/TeleportController/human_markers',
    '/move_humans_node/reset_simulation',
    '/move_humans_node/add_human',
    '/move_humans_node/delete_human',
    '/move_humans_node/add_sub_goal',
    '/move_humans_node/update_goal');
}

class MoveHumansViz {
  ros: ROSLIB.Ros;
  viewer: ROS3DNAV.Viewer;
  viewerDivId: string;
  tfClient: ROSLIB.TFClient;
  gridClient: ROS3DNAV.OccupancyGridClient;

  resetSimulationClient: ROSLIB.Service;
  addHumanClient: ROSLIB.Service;
  deleteHumanClient: ROSLIB.Service;
  addSubgoalClient: ROSLIB.Service;
  updateGoalClient: ROSLIB.Service;

  fixedFrame: string;

  constructor() { }

  initialize(
    rosBridgeUrl: string,
    viewerDivId: string,
    fixedFrame: string,
    mapOpacity: number,
    humanPathsTopic: string,
    humanMarkersTopic: string,
    resetSimulationService: string,
    addHumanService: string,
    deleteHumanService: string,
    addSubgoalService: string,
    updateGoalService: string) {

    // connect to ros
    this.ros = new ROSLIB.Ros({
      url: rosBridgeUrl
    });

    // create the main viewer
    this.viewerDivId = viewerDivId;
    this.viewer = new ROS3DNAV.Viewer({
      divID: this.viewerDivId,
      width: document.getElementById(this.viewerDivId).clientWidth - 15,
      height: window.innerHeight - 18,
      antialias: true,
      cameraPose: { x: 20, y: 10, z: 25 },
    });
    window.addEventListener('resize', this.resizeViewer, false);

    // setup a clietn ot litsen to TFs
    this.tfClient = new ROSLIB.TFClient({
      ros: this.ros,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 30.0,
      fixedFrame: fixedFrame
    });
    this.fixedFrame = fixedFrame;

    // show the map
    this.gridClient = new ROS3DNAV.OccupancyGridClient({
      ros: this.ros,
      viewer: this.viewer,
      rootObject: this.viewer.scene,
      opacity: mapOpacity,
      continuous: false
    });

    // setup human paths clients
    let humanPaths = new ROS3DNAV.PathArray({
      ros: this.ros,
      topic: humanPathsTopic,
      tfClient: this.tfClient,
      rootObject: this.viewer.scene,
      color: 0x1E90FF,
      width: 5
    });

    // setup human marker client
    let humansMarkers = new ROS3D.MarkerArrayClient({
      ros: this.ros,
      topic: humanMarkersTopic,
      tfClient: this.tfClient,
      rootObject: this.viewer.scene
    });

    // setup services
    this.resetSimulationClient = new ROSLIB.Service({
      ros: this.ros,
      name: resetSimulationService,
      serviceType: 'std_srvs/Trigger'
    });
    document.getElementById('reset-simulation-button').onclick = this.resetSimulationRequest;

    this.addHumanClient = new ROSLIB.Service({
      ros: this.ros,
      name: addHumanService,
      serviceType: 'move_humans/HumanUpdate'
    });
    document.getElementById('add-human-button').onclick = this.addHumanClicked;

    this.deleteHumanClient = new ROSLIB.Service({
      ros: this.ros,
      name: deleteHumanService,
      serviceType: 'move_humans/HumanUpdate'
    });

    this.addSubgoalClient = new ROSLIB.Service({
      ros: this.ros,
      name: addSubgoalService,
      serviceType: 'move_humans/HumanUpdate'
    });

    this.updateGoalClient = new ROSLIB.Service({
      ros: this.ros,
      name: updateGoalService,
      serviceType: 'move_humans/HumanUpdate'
    });
  }

  resizeViewer = () => {
    this.viewer.resize(document.getElementById(this.viewerDivId).clientWidth - 15,
      window.innerHeight - 18);
  }

  resetSimulationRequest = () => {
    let req = new StdSrvs.TriggerRequest({});
    this.resetSimulationClient.callService(req, this.resetSimulationResult);
  }
  resetSimulationResult = (res: StdSrvs.TriggerResponse) => {
    if (!res.success) {
      console.log(res.message);
    }
  }

  addHumanClicked = () => {
    this.startArrowControl('add-human-id', this.addHumanRequest);
  }

  deleteHumanClicked = () => {
    this.startArrowControl('delete-human-id', this.deleteHumanRequest);
  }

  addSubGoalClicked = () => {
    this.startArrowControl('add-sub-goal-id', this.addSubgoalRequest);
  }

  updateGoalClicked = () => {
    this.startArrowControl('update-goal-id', this.updateGoalRequest);
  }

  startArrowControl (humanIDField: string, poseReceiveCB: any) {
    let humanId = this.validateId(humanIDField);
    if (humanId === -1) {
      this.setMessage('Invalid human id');
      return;
    }
    if (!this.gridClient.createArrowControl(this.arrowCallback, {id: humanId, poseReceiveCB: poseReceiveCB})) {
      this.setMessage('Please wait, map is being downloaded');
    } else {
      this.setMessage('Please choose a pose on the map');
      this.disableHumanUpdateButtons();
    }
  }

  addHumanRequest = (id: number, pose: THREE.Vector3, direction: THREE.Quaternion) => {
    let req = new MoveHumans.HumanUpdateRequest({
      human_pose: this.createHumanPose(id, pose, direction)
    });
    if (!this.addHumanClient.callService(req, this.humanUpdateResponse)) {
      this.setMessage('Service call returned false');
    }
  }

  deleteHumanRequest = (id: number) => {
    let pose = new THREE.Vector3();
    let direction = new THREE.Quaternion();
    let req = new MoveHumans.HumanUpdateRequest({
      human_pose: this.createHumanPose(id, pose, direction)
    });
    this.deleteHumanClient.callService(req, this.humanUpdateResponse);
  }

  addSubgoalRequest = (id: number, pose: THREE.Vector3, direction: THREE.Quaternion) => {
    let req = new MoveHumans.HumanUpdateRequest({
      human_pose: this.createHumanPose(id, pose, direction)
    });
    this.addSubgoalClient.callService(req, this.humanUpdateResponse);
  }

  updateGoalRequest = (id: number, pose: THREE.Vector3, direction: THREE.Quaternion) => {
    let req = new MoveHumans.HumanUpdateRequest({
      human_pose: this.createHumanPose(id, pose, direction)
    });
    this.updateGoalClient.callService(req, this.humanUpdateResponse);
  }

  humanUpdateResponse = (res: MoveHumans.HumanUpdateResponse) => {
    if (!res.success) {
      console.log(res.message);
    }
  }

  createHumanPose(id: number, pose: THREE.Vector3, direction: THREE.Quaternion) {
    return new MoveHumans.HumanPose({
      human_id: id,
      pose: new GeometryMsgs.PoseStamped({
        header: new StdMsgs.Header({
          seq: 1,
          stamp: new ROS3DNAV.Time().now(),
          frame_id: this.fixedFrame
        }),
        pose: new GeometryMsgs.Pose({
          position: new GeometryMsgs.Point({
            x: pose.x,
            y: pose.y,
            z: pose.z
          }),
          orientation: new GeometryMsgs.Quaternion({
            x: direction.x,
            y: direction.y,
            z: direction.z,
            w: direction.w
          })
        })
      })
    });
  }

  validateId(elementID: string) {
    let input = (<HTMLInputElement>document.getElementById(elementID)).value;
    if (input.length > 0) {
      let value = Number(input);
      if (!isNaN(value) && value % 1 === 0 && value >= 0) {
        return value;
      }
    }
    return -1;
  }

  setMessage(message: string) {
    (<HTMLInputElement>document.getElementById("message-area")).value = message;
  }

  disableHumanUpdateButtons() {
    (<HTMLButtonElement>document.getElementById("add-human-button")).disabled = true;
    (<HTMLButtonElement>document.getElementById("delete-human-button")).disabled = true;
    (<HTMLButtonElement>document.getElementById("add-sub-goal-button")).disabled = true;
    (<HTMLButtonElement>document.getElementById("update-goal-button")).disabled = true;
  }
  enableHumanUpdateButtons() {
    (<HTMLButtonElement>document.getElementById("add-human-button")).disabled = false;
    (<HTMLButtonElement>document.getElementById("delete-human-button")).disabled = false;
    (<HTMLButtonElement>document.getElementById("add-sub-goal-button")).disabled = false;
    (<HTMLButtonElement>document.getElementById("update-goal-button")).disabled = false;
  }

  arrowCallback = (success: boolean, position?: THREE.Vector3, orientation?: THREE.Quaternion, message?: string, senderInfo?: any) => {
    this.enableHumanUpdateButtons();
    if (success) {
      this.setMessage('Point: x=' + position.x + ', y=' + position.y + ', angle=' + orientation);
      senderInfo.poseReceiveCB(senderInfo.id, position, orientation);
    } else {
      this.setMessage(message);
    }
  }

}
