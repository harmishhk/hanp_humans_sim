namespace MoveHumans {
  export class HumanPose extends ROSLIB.Message {
    public human_id: number;
    public pose: GeometryMsgs.PoseStamped;

    constructor(values: {
      human_id: number,
      pose: GeometryMsgs.PoseStamped
    }) {
      super(values);
    }
  }

  export class HumanPoseArray extends ROSLIB.Message {
    public human_id: number;
    public poses: GeometryMsgs.PoseStamped[];

    constructor(values: {
      human_id: number,
      poses: GeometryMsgs.PoseStamped[]
    }) {
      super(values);
    }
  }

  export class HumanUpdateRequest extends ROSLIB.ServiceRequest {
    public human_pose: MoveHumans.HumanPose;

    constructor(values: {
      human_pose: MoveHumans.HumanPose
    }) {
      super(values);
    }
  }

  export class HumanUpdateResponse extends ROSLIB.ServiceResponse {
    public success: boolean;
    public message: string;

    constructor(values: {
      success: boolean;
      message: string;
    }) {
      super(values);
    }
  }
}

declare module "MoveHumans" {
  export = MoveHumans;
}
