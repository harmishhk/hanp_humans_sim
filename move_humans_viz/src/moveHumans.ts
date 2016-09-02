namespace MoveHumans {
  export class HumanPose extends ROSLIB.Message {
    human_id: number;
    pose: GeometryMsgs.PoseStamped;

    constructor(values: {
      human_id: number,
      pose: GeometryMsgs.PoseStamped
    }) {
      super(values);
    }
  }

  export class HumanPoseArray extends ROSLIB.Message {
    human_id: number;
    poses: GeometryMsgs.PoseStamped[];

    constructor(values: {
      human_id: number,
      poses: GeometryMsgs.PoseStamped[]
    }) {
      super(values);
    }
  }

  export class HumanUpdateRequest extends ROSLIB.ServiceRequest {
    human_pose: MoveHumans.HumanPose;

    constructor(values: { human_pose: MoveHumans.HumanPose }) {
      super(values);
    }
  }

  export class HumanUpdateResponse extends ROSLIB.ServiceResponse {
    success: boolean;
    message: string;

    constructor(values: {
      success: boolean;
      message: string;
    }) {
      super(values);
    }
  }
}

declare module 'MoveHumans' {
  export = MoveHumans;
}
