package strapDown;


public class ThreeAxis {
    public double x;
    public double y;
    public double z;
    void print(){
    	System.out.println(x+","+y+","+z);
    }
    public void showDeg(){
    	System.out.println((x/Math.PI*180)+","+(y/Math.PI*180)+","+(z/Math.PI*180));
    }
    ThreeAxis crossProduct(ThreeAxis a,ThreeAxis b){
    	ThreeAxis c = new ThreeAxis();
//    	BigDecimal ax = new BigDecimal(a.x);
//    	BigDecimal ay = new BigDecimal(a.y);
//    	BigDecimal az = new BigDecimal(a.z);
//    	BigDecimal bx = new BigDecimal(b.x);
//    	BigDecimal by = new BigDecimal(b.y);
//    	BigDecimal bz = new BigDecimal(b.z);
//    	BigDecimal tmp =new BigDecimal(0);
//    	tmp.add(ay.multiply(bz));
//    	tmp. minus(az.multiply(by);
//
    	c.x = a.y * b.z - a.z * b.y;
    	c.y = a.z * b.x - a.x * b.z;
    	c.z = a.x * b.y - a.y * b.x;
    	return c;
    }
    ThreeAxis mul(double a){
    	ThreeAxis b = new ThreeAxis();
    	b.x = this.x * a;
    	b.y = this.y * a;
    	b.z = this.z * a;
    	return b;
    }
    ThreeAxis plus(ThreeAxis a){
    	ThreeAxis b = new ThreeAxis();
    	b.x = this.x + a.x;
    	b.y = this.y + a.y;
    	b.z = this.z + a.z;
    	return b;
    }
    ThreeAxis minus(ThreeAxis a){
    	ThreeAxis b = new ThreeAxis();
    	b.x = this.x - a.x;
    	b.y = this.y - a.y;
    	b.z = this.z - a.z;
    	return b;
    }
    double norm(){
    	return Math.sqrt(this.x*this.x + this.y*this.y + this.z*this.z);
    }
    double[] matrix(){
    	double[] a = new double[3];
    	a[0] = x;
    	a[1] = y;
    	a[2] = z;
    	return a;
    }
}
