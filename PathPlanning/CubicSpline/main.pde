Spline2D spline2D; 
Utils utils = new Utils();
PVector offset ;
ArrayList<Float> rx = new ArrayList<Float>();
ArrayList<Float> ry = new ArrayList<Float>();
ArrayList<Double> ryaw = new ArrayList<Double>();
ArrayList<Double> rk = new ArrayList<Double>();
void setup() {
  size(1000, 1000);
  offset = new PVector(width/2, height/2);

  float [] x = {-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0};
  float [] y = {0.7, -6.0, 5.0, 6.5, 0.0, 5.0, -2.0};

  spline2D = new Spline2D(x,y);
  float [] s = utils.arange(0.0,spline2D.s[spline2D.s.length-1],0.1);
 
  for(float val : s){
    rx.add(spline2D.calc_position(val)[0]);
    ry.add(spline2D.calc_position(val)[1]);
    ryaw.add(spline2D.calc_yaw(val));
    rk.add(spline2D.calc_curvature(val));
  }


}

void draw(){
  pushMatrix(); 
  translate(offset.x, offset.y);
  for(int i = 0;i < rx.size(); ++i){
    point(rx.get(i)*10,-ry.get(i)*10);
  }
  popMatrix();
}
