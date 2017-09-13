package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 20;
  private static final int PROPCONST = 5;
  private static final int MAXCORRECTION = 100;

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;
  private int distError;
  

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {
	  this.distError = bandCenter - distance;
	  int difference;
    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    if (distance >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 255) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }
    
    
    if(Math.abs(distError) < bandWidth){
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    	
    }
    else if(distError>0){
    	difference = calcProp(distError);
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED - difference);
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + difference);
    	
    }
    else if(distError<0){
    	difference = calcProp(distError);
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + difference);
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - difference);
    }
    
    
    

    // TODO: process a movement based on the us distance passed in (P style)
  }
  
  int calcProp(int difference){
	  int correction;
	  distError = Math.abs(distError);
	  
	  correction = (int)(PROPCONST * (double)distError);
	  
	  if(correction >= MOTOR_SPEED){
		  correction = MAXCORRECTION;
	  }
	  
	  return correction;
  }


  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
