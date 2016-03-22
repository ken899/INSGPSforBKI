package strapDown;

class Motorcycle extends IMU{
	ThreeAxis v;//速度
	ThreeAxis gyro;//角度．x:roll,y:pitch,z:yaw
	ThreeAxis pos;//位置 x:lon,y:lat,z:height
	double wander_angle;//wander_angle
	Motorcycle(){
		v = new ThreeAxis();
		gyro = new ThreeAxis();
		pos = new ThreeAxis();
	}
	void show(){
		System.out.println(",x,y,z");
		System.out.println("v," + v.x + "," + v.y + "," +v.z);
		System.out.println("pos," + pos.x + "," + pos.y + "," +pos.z);
//		System.out.println("acc," + acc.x + "," + acc.y + "," +acc.z);
//		System.out.println("gyro," + gyro.x + "," + gyro.y + "," +gyro.z);
	}
	void showSpeedHour(){
    	System.out.println((v.x*3.6)+","+(v.y*3.6)+","+(v.z*3.6));
	}
	void showGyro(){
		System.out.println((gyro_v.x * 180 / Math.PI) + "," + (gyro_v.y * 180 / Math.PI) + "," + ( gyro_v.z * 180 / Math.PI));
	}
	void pshow(){
		System.out.println("" + pos.x + "," + pos.y + "," + pos.z);
	}
	void pshowR(){
		System.out.println("" + (pos.x * 180 / Math.PI) + "," + ( pos.y * 180 / Math.PI) + "," + pos.z);
	}

}