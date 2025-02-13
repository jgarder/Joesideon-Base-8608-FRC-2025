package frc.robot.AlphaBots;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AprilTag
{   
    public enum TagType
    {
      none,
      Source,
      Reef,
      Processor,
      BlueBarge,
      RedBarge,

    }
    public int ID = 0;
    public String name ="";
    public Pose2d Pose = new Pose2d(0, 0, new Rotation2d(0));
    public double yRotatinDegrees = 0;
    public boolean algaeOnUpper = false;
    public double extraOffsetWhenTargeting = 0;//(+) away from tag (-) towards the tag (from robot view)
    public double offset90Offset = 0;//Positive number will spread left and right further from center tag point.
    public TagType tagType = TagType.none;
    public AprilTag(int _ID,String _name, Pose2d _position,  double _yRot)
    {
      ID = _ID;
      name =_name;
      Pose = _position;
      yRotatinDegrees = _yRot;
    }

    public AprilTag Withoffset90(double extraOffset)
    {
      offset90Offset = extraOffset;
      return this;
    }
    public AprilTag Withdepthoffset(double extraOffset)
    {
      extraOffsetWhenTargeting = extraOffset;
      return this;
    }
    public AprilTag WithAlgaeOnUpper()
    {
      algaeOnUpper = true;
      return this;
    }
    public AprilTag WithType(TagType ThisType)
    {
      tagType = ThisType;
      return this;
    }

}