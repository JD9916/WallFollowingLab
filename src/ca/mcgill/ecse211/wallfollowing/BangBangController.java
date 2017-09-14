package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  private int distance;
  private int distError;
  private int filterControl;
  public static final int FILTER_OUT = 20;
  
  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
	this.filterControl = 0;
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {
	  
	  if (distance >= 255 && filterControl < FILTER_OUT) {
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
    //this.distance = (int)(distance/(Math.sqrt(2.0)));
    this.distError = bandCenter - distance;
    
    if ( (distError <= bandwidth) && (distError >= (0-bandwidth)) ){
    	WallFollowingLab.leftMotor.setSpeed(motorHigh);
    	WallFollowingLab.rightMotor.setSpeed(motorHigh);
    }
    else if (distError > 0) {
    	WallFollowingLab.rightMotor.setSpeed(motorLow);
    	WallFollowingLab.leftMotor.setSpeed(motorHigh);
    }
    else if (distError < 0) {
    	WallFollowingLab.rightMotor.setSpeed(motorHigh);
    	WallFollowingLab.leftMotor.setSpeed(motorLow);
    }
    // TODO: process a movement based on the us distance passed in (BANG-BANG style)
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
