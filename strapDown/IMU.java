package strapDown;

class IMU{
	ThreeAxis acc;
	ThreeAxis gyro_v;//x:roll,y:pitch,z:yaw
	IMU(){
		acc = new ThreeAxis();
		gyro_v = new ThreeAxis();
	}
	void show(){
		System.out.print("acc,");
		acc.print();
		System.out.print("gyro,");
		gyro_v.print();
	}
}