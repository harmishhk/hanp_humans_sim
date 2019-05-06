function init() {
  let moveHumansViz = new MoveHumansViz(
    "ws://localhost:9090",
    "viewer",
    "map",
    0.7,
    "/move_humans_node/TeleportController/plans",
    "/move_humans_node/human_markers",
    "/move_humans_node/reset_simulation",
    "/move_humans_node/add_human",
    "/move_humans_node/delete_human",
    "/move_humans_node/add_sub_goal",
    "/move_humans_node/update_goal");
}

class MoveHumansViz {
  private ros: ROSLIB.Ros;
  private viewer: ROS3DNAV.Viewer;
  private viewerDivId: string;
  private tfClient: ROSLIB.TFClient;
  private gridClient: ROS3DNAV.OccupancyGridClient;

  private resetSimulationClient: ROSLIB.Service;
  private addHumanClient: ROSLIB.Service;
  private deleteHumanClient: ROSLIB.Service;
  private addSubgoalClient: ROSLIB.Service;
  private updateGoalClient: ROSLIB.Service;

  private fixedFrame: string;

  constructor(
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
      url: rosBridgeUrl,
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
    window.addEventListener("resize", this.resizeViewer, false);

    // setup a clietn ot litsen to TFs
    this.tfClient = new ROSLIB.TFClient({
      ros: this.ros,
      fixedFrame: fixedFrame,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 30.0,
    });
    this.fixedFrame = fixedFrame;

    // show the map
    this.gridClient = new ROS3DNAV.OccupancyGridClient({
      ros: this.ros,
      viewer: this.viewer,
      continuous: false,
      rootObject: this.viewer.scene,
      opacity: mapOpacity,
    });

    // setup human paths clients
    let humanPaths = new ROS3DNAV.HumanPathArray({
      ros: this.ros,
      topic: humanPathsTopic,
      tfClient: this.tfClient,
      rootObject: this.viewer.scene,
      color: 0x1E90FF,
      width: 5,
    });

    // setup human marker client
    let humansMarkers = new ROS3D.MarkerArrayClient({
      ros: this.ros,
      topic: humanMarkersTopic,
      tfClient: this.tfClient,
      rootObject: this.viewer.scene,
    });

    // setup services
    this.resetSimulationClient = new ROSLIB.Service({
      ros: this.ros,
      name: resetSimulationService,
      serviceType: "std_srvs/Trigger",
    });
    document.getElementById("reset-simulation-button").onclick = this.resetSimulationClicked;

    this.addHumanClient = new ROSLIB.Service({
      ros: this.ros,
      name: addHumanService,
      serviceType: "move_humans/HumanUpdate",
    });
    document.getElementById("add-human-button").onclick = this.addHumanClicked;

    this.deleteHumanClient = new ROSLIB.Service({
      ros: this.ros,
      name: deleteHumanService,
      serviceType: "move_humans/HumanUpdate",
    });
    document.getElementById("delete-human-button").onclick = this.deleteHumanClicked;

    this.addSubgoalClient = new ROSLIB.Service({
      ros: this.ros,
      name: addSubgoalService,
      serviceType: "move_humans/HumanUpdate",
    });
    document.getElementById("add-sub-goal-button").onclick = this.addSubGoalClicked;

    this.updateGoalClient = new ROSLIB.Service({
      ros: this.ros,
      name: updateGoalService,
      serviceType: "move_humans/HumanUpdate",
    });
    document.getElementById("update-goal-button").onclick = this.updateGoalClicked;
  }

  public resizeViewer = () => {
    this.viewer.resize(document.getElementById(this.viewerDivId).clientWidth - 15,
      window.innerHeight - 18);
  }

  private resetSimulationClicked = () => {
    let req = new StdSrvs.TriggerRequest({});
    this.resetSimulationClient.callService(req, this.resetSimulationResponse);
  }

  private addHumanClicked = () => {
    this.startArrowControl("add-human-id", this.addHumanRequest);
  }

  private deleteHumanClicked = () => {
    let humanId = this.validateId("delete-human-id");
    if (humanId !== -1) {
      this.deleteHumanRequest(humanId);
    }
  }

  private addSubGoalClicked = () => {
    this.startArrowControl("add-sub-goal-id", this.addSubgoalRequest);
  }

  private updateGoalClicked = () => {
    this.startArrowControl("update-goal-id", this.updateGoalRequest);
  }

  private resetSimulationResponse = (res: StdSrvs.TriggerResponse) => {
    if (!res.success) {
      this.setMessage(res.message);
    }
  }

  private startArrowControl(humanIDField: string, poseReceiveCB: any) {
    let humanId = this.validateId(humanIDField);
    if (humanId !== -1) {
      if (!this.gridClient.createArrowControl(this.arrowCallback, { id: humanId, poseReceiveCB: poseReceiveCB })) {
        this.setMessage("Please wait, map is being downloaded");
      } else {
        this.setMessage("Please choose a pose on the map");
        this.disableHumanUpdateButtons();
      }
    }
  }

  private addHumanRequest = (id: number, pose: THREE.Vector3, direction: THREE.Quaternion) => {
    let req = new MoveHumans.HumanUpdateRequest({
      human_pose: this.createHumanPose(id, pose, direction),
    });
    if (!this.addHumanClient.callService(req, this.humanUpdateResponse)) {
      this.setMessage("Service call returned false");
    }
  }

  private deleteHumanRequest = (id: number) => {
    let pose = new THREE.Vector3();
    let direction = new THREE.Quaternion();
    let req = new MoveHumans.HumanUpdateRequest({
      human_pose: this.createHumanPose(id, pose, direction),
    });
    this.deleteHumanClient.callService(req, this.humanUpdateResponse);
  }

  private addSubgoalRequest = (id: number, pose: THREE.Vector3, direction: THREE.Quaternion) => {
    let req = new MoveHumans.HumanUpdateRequest({
      human_pose: this.createHumanPose(id, pose, direction),
    });
    this.addSubgoalClient.callService(req, this.humanUpdateResponse);
  }

  private updateGoalRequest = (id: number, pose: THREE.Vector3, direction: THREE.Quaternion) => {
    let req = new MoveHumans.HumanUpdateRequest({
      human_pose: this.createHumanPose(id, pose, direction),
    });
    this.updateGoalClient.callService(req, this.humanUpdateResponse);
  }

  private arrowCallback = (success: boolean, position?: THREE.Vector3, orientation?: THREE.Quaternion,
    message?: string, senderInfo?: any) => {
    this.enableHumanUpdateButtons();
    if (success) {
      this.setMessage("Point: x=" + position.x + ", y=" + position.y + ", angle=" + orientation);
      senderInfo.poseReceiveCB(senderInfo.id, position, orientation);
    } else {
      this.setMessage(message);
    }
  }

  private humanUpdateResponse = (res: MoveHumans.HumanUpdateResponse) => {
    if (!res.success) {
      this.setMessage("Human update service failed:\n" + res.message);
    } else {
      this.setMessage(res.message);
    }
  }


  private createHumanPose(id: number, pose: THREE.Vector3, direction: THREE.Quaternion) {
    return new MoveHumans.HumanPose({
      human_id: id,
      pose: new GeometryMsgs.PoseStamped({
        header: new StdMsgs.Header({
          seq: 1,
          stamp: ROS3DNAV.Time.now(),
          frame_id: this.fixedFrame,
        }),
        pose: new GeometryMsgs.Pose({
          position: new GeometryMsgs.Point({
            x: pose.x,
            y: pose.y,
            z: pose.z,
          }),
          orientation: new GeometryMsgs.Quaternion({
            x: direction.x,
            y: direction.y,
            z: direction.z,
            w: direction.w,
          }),
        }),
      }),
    });
  }

  private setMessage(message: string) {
    (<HTMLInputElement>document.getElementById("message-area")).value = message;
  }

  private validateId(elementID: string) {
    let input = (<HTMLInputElement>document.getElementById(elementID)).value;
    if (input.length > 0) {
      let value = Number(input);
      if (!isNaN(value) && value % 1 === 0 && value >= 0) {
        return value;
      }
    }
    this.setMessage("Invalid human id");
    return -1;
  }

  private disableHumanUpdateButtons() {
    (<HTMLButtonElement>document.getElementById("add-human-button")).disabled = true;
    (<HTMLButtonElement>document.getElementById("delete-human-button")).disabled = true;
    (<HTMLButtonElement>document.getElementById("add-sub-goal-button")).disabled = true;
    (<HTMLButtonElement>document.getElementById("update-goal-button")).disabled = true;
  }

  private enableHumanUpdateButtons() {
    (<HTMLButtonElement>document.getElementById("add-human-button")).disabled = false;
    (<HTMLButtonElement>document.getElementById("delete-human-button")).disabled = false;
    (<HTMLButtonElement>document.getElementById("add-sub-goal-button")).disabled = false;
    (<HTMLButtonElement>document.getElementById("update-goal-button")).disabled = false;
  }
}
