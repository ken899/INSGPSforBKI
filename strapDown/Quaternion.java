package strapDown;
//クォータニオン
public class Quaternion extends ThreeAxis{
	public double w;
	void assignmentThreeAxis(ThreeAxis a){
		//三軸をクォータニオンに変換
		this.x = a.x;
		this.y = a.y;
		this.z = a.z;
	}
	public void print(){
		//クォータニオンを表示
		System.out.println(w + "," + x + "," + y + "," + z);
	}
	Quaternion plus(ThreeAxis t){
		//三軸と加算
		Quaternion a = new Quaternion();
		a.w = this.w;
		a.x = this.x + t.x;
		a.y = this.y + t.y;
		a.z = this.z + t.z;
		return a;
	}
	Quaternion plus(Quaternion q){
		//クォータニオンと加算
		Quaternion a = new Quaternion();
		a.w = this.w + q.w;
		a.x = this.x + q.x;
		a.y = this.y + q.y;
		a.z = this.z + q.z;
		return a;
	}
	Quaternion minus(ThreeAxis t){
		//三軸と減算
		Quaternion a = new Quaternion();
		a.w = this.w;
		a.x = this.x - t.x;
		a.y = this.y - t.y;
		a.z = this.z - t.z;
		return a;
	}
	Quaternion minus(Quaternion q){
		//クォータニオンと減算
		Quaternion a = new Quaternion();
		a.w = this.w - q.w;
		a.x = this.x - q.x;
		a.y = this.y - q.y;
		a.z = this.z - q.z;
		return a;
	}
	Quaternion mul(double a){
		//スカラー倍
		Quaternion b = new Quaternion();
		b.w = a * this.w;
		b.x = a * this.x;
		b.y = a * this.y;
		b.z = a * this.z;
		return b;
	}
	Quaternion mul(Quaternion q1,Quaternion q2){
		//クォータニオン同士の積算
		Quaternion q = new Quaternion();
		q.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
		q.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
		q.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
		q.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
		return q;
	}
	Quaternion mul(Quaternion q1,ThreeAxis q2){
		//クォータニオンと三軸の積算
		Quaternion q = new Quaternion();
		q.w =-q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
		q.x = q1.w * q2.x + q1.y * q2.z - q1.z * q2.y;
		q.y = q1.w * q2.y - q1.x * q2.z + q1.z * q2.x;
		q.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x;
		return q;
	}
	Quaternion conjugate(Quaternion q){
		//共役
		q.w =  q.w;
		q.x = -q.x;
		q.y = -q.y;
		q.z = -q.z;
		return q;
	}
	public Quaternion conjugate(){
		//共役
		Quaternion a = new Quaternion();
		a.w =  this.w;
		a.x = -this.x;
		a.y = -this.y;
		a.z = -this.z;
		return a;
	}
	public ThreeAxis rotateFrame(Quaternion q,ThreeAxis t){
		//座標変換
		Quaternion result = new Quaternion();
		q = q.regularize();
		result = mul(q.conjugate(), t);
		result = mul(result,q);
		return result;
	}
	public Quaternion rotateFrame(ThreeAxis t){
		//座標変換
		Quaternion result = new Quaternion();
		Quaternion q = this.regularize();
		result = mul(q.conjugate(), t);
		result = mul(result,q);
		return result;
	}
	public Quaternion rotateFrame(Quaternion q,Quaternion t){
		//座標変換
		Quaternion result = new Quaternion();
		q = q.regularize();
		result = mul(q.conjugate(), t);
		result = mul(result,q);
		return result;
	}
	public Quaternion rotateFrame(Quaternion t){
		//座標変換
		Quaternion result = new Quaternion();
		Quaternion q = this.regularize();
		result = mul(q.conjugate(), t);
		result = mul(result,q);
		return result;
	}
	Quaternion regularize(){
		//正規化
		Quaternion a = new Quaternion();
		a.w = this.w / this.norm();
		a.x = this.x / this.norm();
		a.y = this.y / this.norm();
		a.z = this.z / this.norm();
		return a;
	}
	double norm(){
		//距離
		double a;
		a = Math.sqrt(this.w*this.w + this.x*this.x + this.y*this.y + this.z*this.z);
		return a;
	}
	double[][] getDCM(){
		//DCM方式座標変換の行列に変換
		double[][] a = new double[3][3];
		a[0][0] = w*w + x*x - y*y - z*z;
		a[0][1] = 2 * (x*y + w*z);
		a[0][2] = 2 * (x*z - w*y);
		a[1][0] = 2 * (x*y - w*z);
		a[1][1] = w*w - x*x + y*y - z*z;
		a[1][2] = 2 * (y*z + w*x);
		a[2][0] = 2 * (x*z + w*y);
		a[2][1] = 2 * (y*z - w*x);
		a[2][2] = w*w - x*x - y*y + z*z;
		return a;
	}
}