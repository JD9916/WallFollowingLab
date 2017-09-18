package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 250;
  private static final int FILTER_OUT = 20;
  private static final int PROPCONST = 7;
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

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initialize motor rolling forward
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
    	this.distance = (int) (distance/(Math.sqrt(2.0)));
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = (int) (distance/(Math.sqrt(2.0)));
    }
    
    
    if(Math.abs(distError) <= bandWidth){
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    	
    }
    else if(distError>0){
    	difference = calcProp(distError);
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED - difference);
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + difference);
    	if(difference >= MAXCORRECTION) {
    		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + difference);
    		WallFollowingLab.leftMotor.forward();
    		WallFollowingLab.rightMotor.backward();
    	}
    	else {
    		WallFollowingLab.leftMotor.forward();
    		WallFollowingLab.rightMotor.forward();
    	}
    }
    else if(distError<0){
    	difference = calcProp(distError);
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + difference);
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - difference);
    	//if(distance > (0-255)) {
    		//WallFollowingLab.leftMotor.backward();
    		//WallFollowingLab.rightMotor.forward();
    	//}
    	//else {
    		WallFollowingLab.leftMotor.forward();
    		WallFollowingLab.rightMotor.forward();
    	//}   
    }
    
    
    

    // TODO: process a movement based on the us distance passed in (P style)
  }
  
  int calcProp(int difference){
	  int correction;
	  difference = Math.abs(difference);
	  
	  correction = (int)(PROPCONST * (double)difference);
	  
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
