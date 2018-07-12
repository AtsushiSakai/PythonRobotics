class Spline {
  
  float[] x,y;
  Utils utils = new Utils();
  ArrayList<Float> d = new ArrayList<Float>();
  ArrayList<Float> b = new ArrayList<Float>();
  double[] c;
  
  Spline(float[] x, float[] y ){
     this.x = x;
     this.y = y;
     float[] h = utils.diff(x);
     float[][] A = __calc_A(h); 
     float[] B = __calc_B(h);
     Utils.Matrix matrix = utils.new Matrix(A);
     //matrix.mPrint();
     c = matrix.solve(B,true);
     for (int i = 0; i < x.length-1; i++){
       d.add((float)(c[i+1] - c[i]) / (3.0 * h[i]));
       float tb =(float)((y[i + 1] - y[i]) / h[i] - h[i] * 
                (c[i + 1] + 2.0 * c[i]) / 3.0);
       b.add(tb);
     }
  }
  
     public float calc(float t) {
       if (t < this.x[0]) return 0.0;
       else if(t > this.x[x.length-1]) return 0.0;
       int i = __search_index(t);
       float dx = t - this.x[i];
       float result = this.y[i] + this.b.get(i) * dx
             + (float)this.c[i] * pow(dx,2) + this.d.get(i) * pow(dx,3);
       return result;
     }
     
     public float calcd(float t){
       if (t < this.x[0]) return 0.0;
       else if(t > this.x[x.length-1]) return 0.0;
       int i = __search_index(t);
       float dx = t - this.x[i];
       float result = this.b.get(i) + 2.0 * (float)this.c[i] * dx + 3.0 
             * this.d.get(i) * pow(dx, 2);
       return result;
     }
     
     public float calcdd(float t){
       if (t < this.x[0]) return 0.0;
       else if(t > this.x[x.length-1]) return 0.0;
       int i = __search_index(t);
       float dx = t - this.x[i];
       float result = 2.0 * (float)this.c[i] + 6.0 * this.d.get(i) * dx;
       return result;
     }
     
     private float[][] __calc_A(float[] h){
        int nx = h.length + 1;
        float[][] A = new float[nx][nx];
        A[0][0] = 1;
        
        for(int i = 0; i < nx - 1; i++){
          if(i != (nx - 2))
            A[i + 1][i + 1] = 2.0 * (h[i] + h[i + 1]);
          A[i + 1][i] = h[i];
          A[i][i + 1] = h[i];
        }
        
        A[0][1] = 0.0;
        A[nx - 1][nx - 2] = 0.0;
        A[nx - 1][nx - 1] = 1.0;
       
        //utils.new Matrix(A).mPrint();
        return A;
     }
        
    private float[] __calc_B(float[] h){
        int nx = h.length + 1;
        float[] B = new float[nx];
        for(int i = 0; i < nx - 2; i++){
            B[i + 1] = 3.0 * (y[i + 2] - y[i + 1]) /
                h[i + 1] - 3.0 * (y[i + 1] - y[i]) / h[i];
        }
        //print(" "+B[i]);
        return B;
    }
    
    private int __search_index(float val){
    return utils.bisect_bisect(val, this.x) - 1;
    }
}

class Spline2D{
   
  float[] s;
  Utils utils = new Utils();
  Spline sx, sy;
  
  Spline2D(float[] x, float[] y){
    this.s = __calc_s(x, y);
    sx = new Spline(this.s, x);
    sy = new Spline(this.s, y);
  }
  
  private float[] __calc_s(float[] x, float[] y){
    float[] dx = utils.diff(x);
    float[] dy = utils.diff(y);
    float[] ds = new float[dx.length];
    for(int i = 0; i < dx.length; i++){
      ds[i] = sqrt(pow(dx[i], 2) + pow(dy[i],2));
    }  
    float[] s = {0};
    ds = utils.extend(s, utils.cumsum(ds));
    return ds;
  }
  
  public float[] calc_position(float s){
    float x = sx.calc(s);
    float y = sy.calc(s);
    return new float[] {x,y};
  }
  
  public double calc_yaw(float s){
    float dx = sx.calcd(s);
    float dy = sy.calcd(s);
    double yaw = atan2(dy,dx);
    return yaw;
  }
  
  public double calc_curvature(float s){
    double dx  = sx.calcd(s);
    double ddx = sx.calcdd(s); 
    double dy  = sy.calcd(s);
    double ddy = sy.calcdd(s);
    double k   = (ddy *dx - ddx * dy) / (Math.pow((Math.pow(dx, 2.0) + Math.pow(dy, 2.0)), 3 / 2));
    return k;
  }
}
