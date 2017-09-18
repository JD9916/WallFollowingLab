package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 250;
  private static final int FILTER_OUT = 20;       // Number of US samples at high distance before the distance is registered
  private static final int PROPCONST = 7;         // Constant that determines how much correction is needed for a certain deviation form the band center
  private static final int MAXCORRECTION = 100;   // The correction applied when the calculated correction exceeds the motor speed

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;
  private int distError;     // Difference between the band center and the actual distance
  

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
    
    
    if(Math.abs(distError) <= bandWidth){   // When the deviation from the band center is within the bandwidth, keep the robot moving forward
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    	
    }
    else if(distError>0){  // When the deviation from the band center is outside the bandwidth and is positive, turn the robot right
    	difference = calcProp(distError);
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED - difference);
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + difference);
    	if(difference >= MAXCORRECTION) {   // Rotate the robot clockwise in its position if it gets too close to the wall
    		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + difference);
    		WallFollowingLab.leftMotor.forward();
    		WallFollowingLab.rightMotor.backward();
    	}
    	else { 
    		WallFollowingLab.leftMotor.forward();
    		WallFollowingLab.rightMotor.forward();
    	}
    }
    else if(distError<0){  // When the deviation from the band center is outside the bandwidth and is negative, turn the robot left
    	difference = calcProp(distError);
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED + difference);
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - difference);
   		WallFollowingLab.leftMotor.forward();
   		WallFollowingLab.rightMotor.forward();
    }
    
    
    

    // TODO: process a movement based on the us distance passed in (P style)
  }
  
  int calcProp(int difference){   // Calculates the correction needed according to the amount of deviation from the band center
	  int correction;   // The correction to the motor speed
	  difference = Math.abs(difference);   
	  
	  correction = (int)(PROPCONST * (double)difference);  // The correction is calculated by multiplying the proportionality constant by the deviation from the band center
	  
	  if(correction >= MOTOR_SPEED){   // Set the correction to the max correction variable if the calculated value exceeds the motor speed
		  correction = MAXCORRECTION;
	  }
	  
	  return correction;
  }


  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
