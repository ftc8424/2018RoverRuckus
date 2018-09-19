package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware4Motor extends Hardware {

    /* These are values that can be used by the opModes (e.g., robot.LBack.setPower(); */
    public DcMotor LFront = null;
    public DcMotor RFront = null;

    /**
     * Constructor for Hardware types, takes the HardwareMap from the OpMode and saves it for later.
     * Each sub-class should call super.Hardware(hw) in case any of the classes in the hierarchy
     * wants to do something at construction time.
     *
     * @param hw The hardware map we'll use for this particular OpMode run
     */
    public Hardware4Motor(HardwareMap hw) {
        super(hw);
    }

    /**
     * Configure and initialize the two back motors, with no reversal.  SUB-CLASS MUST OVERRIDE.
     * <p>
     * Polymorphism will cause sub-classes to call theirs and they must then call super.initMotor()
     * to get me to be called.
     */
    @Override
    public void initMotor() {
        initMotor(false);
    }

    /**
     * Configure and Initialize the two back motors, reversing the left side if revLeft is true, SUBCLASS MUST OVERRIDE.
     * <p>
     * Note, polymorphism will cause my sub-classes to call theirs with an override and then
     * they'll call super.initMotor().  If the type of robot is using a normal drive, then the left
     * side motors will be set to REVERSE so that giving them the positive values moves the robot
     * forward.  In those cases, initMotor(true) should be called.
     * <p>
     * If this is a holonomic drive (e.g., Mecanum or omni-wheel in crab-drive format) then you
     * don't want to do that, so pass with false (or just call the version without parameters).
     *
     * @param revLeft Should I reverse the left motors?
     */
    @Override
    public void initMotor(boolean revLeft) {
        LFront = hwMap.dcMotor.get(Constants.LFRONT);
        RFront = hwMap.dcMotor.get(Constants.RFRONT);

        RFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (revLeft)
            LFront.setDirection(DcMotorSimple.Direction.REVERSE);

        this.setEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // Default to no encoders
        LFront.setPower(0);
        RFront.setPower(0);

        super.initMotor(revLeft);
    }

    /**
     * Set the Encoder mode for the base class's motors.  This should be overridden in the
     * sub-classes and they should call super.setEncoderMode() if they need to set a value.
     *
     * @param mode
     */
    @Override
    public void setEncoderMode(DcMotor.RunMode mode) {
        LFront.setMode(mode);
        RFront.setMode(mode);
        super.setEncoderMode(mode);
    }
}
