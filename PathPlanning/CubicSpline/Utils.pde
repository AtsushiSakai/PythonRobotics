public class Utils {
  Utils() {
  }


  public float[] diff(float[] x) {
    int nx = x.length - 1; 
    float[] h = new float[nx];

    for (int i = 0; i < nx; ++i) {
      h[i] = x[i+1] - x[i];
    }
    return h;
  }
  
  public float[] arrayListToArray(ArrayList<Float> arrayList){
    float[] array = new float[arrayList.size()];
    int i = 0;
    for(float x : arrayList){
      array[i] = x;
      ++i;
    }
    return array;
  }

  public  float[] arange(float start, float stop, float step) {
    ArrayList<Float> output = new ArrayList<Float>();
    for (Float i = start; i < stop; i+=step) {
      output.add(i);
    }
    float[] arrayOut = arrayListToArray(output);
    return arrayOut;
  }

  public float[]  cumsum(float[] vector){
    float[] output = new float[vector.length];
    output[0] = vector[0];
    for(int i = 1; i < (vector.length); ++i){
      output[i] = output[i-1] + vector[i];
    }
    return output;
  }
  
  public float[] extend(float[] A, float[] B){

    float[] output = new float[A.length + B.length];
    ArrayList<Float> aux = new ArrayList<Float>();
    for(float x : A){
      aux.add(x);
    }
   
   for(float x : B){
      aux.add(x);
    }
   output =arrayListToArray(aux);
   return output;
  }
  
  public int bisect_bisect(float value, float[] x){
    int i;
    for(i = 0; i < x.length; ++i){
      if(value < x[i]){
        return i; 
      }
    }
    return i;
  }

  public class Matrix {
    double[][] matrix, origMat;
    int[] piv;
    int pivsign = 1, numRows, numColumns;
    Matrix(float[][] mat) {
      this.matrix = floatToDouble(mat);
      this.origMat = floatToDouble(mat);
      numRows = matrix.length;
      numColumns = matrix[0].length;
      piv = new int[numRows];
      for (int i = 0; i < numRows; i++) {
        piv[i] = i;
      }
      pivsign = 1;
      double[] rowi;
      double[] colj = new double[numRows];

      for (int j = 0; j < numColumns; j++) {
        for (int i = 0; i < numRows; i++) {
          colj[i] = matrix[i][j];
        }

        for (int i = 0; i < numRows; i++) {
          rowi = matrix[i];
          int kmax = Math.min(i, j);
          float s = 0.0f;
          for (int k = 0; k < kmax; k++) {
            s += rowi[k] * colj[k];
          }
          rowi[j] = colj[i] -= s;
        }

        int p = j;
        for (int i = j+1; i < numRows; i++) {
          if (Math.abs(colj[i]) > Math.abs(colj[p])) {
            p = i;
          }
        }
        if (p != j) {
          for (int k = 0; k < numColumns; k++) {
            double t = matrix[p][k]; 
            matrix[p][k] = matrix[j][k]; 
            matrix[j][k] = t;
          }
          int k = piv[p]; 
          piv[p] = piv[j]; 
          piv[j] = k;
          pivsign = -pivsign;
        }

        if (j < numRows & matrix[j][j] != 0.0) {
          for (int i = j+1; i < numRows; i++) {
            matrix[i][j] /= matrix[j][j];
          }
        }
      }
    } 
    public void mPrint() {
      for (int i = 0; i < origMat.length; i++) {
        for (int j = 0; j < origMat.length; j++) {
          print(" " + origMat[i][j]+",");
        }
        println(";");
      }
    }

    public double[][] floatToDouble(float[][] _inputMatrix) {
      int numColumns = _inputMatrix[0].length; 
      int numRows = _inputMatrix.length;
      double[][] outputMatrix = new double[numRows][numColumns];
      for (int i = 0; i < numRows; i++) {
        for (int j = 0; j < numColumns; j++) {
          outputMatrix[i][j] = (double)_inputMatrix[i][j];
        }
      }    
      return outputMatrix;
    }

    public double[] floatToDouble(float[] _inputVec) {
      int numRows = _inputVec.length;
      double[] outputMatrix = new double[numRows];
      for (int i = 0; i < numRows; i++) {
        outputMatrix[i] = (double)_inputVec[i];
      }    
      return outputMatrix;
    }

    public float[][] mCopy() {
      int numRows = matrix.length; 
      int numColumns = matrix[0].length;
      float[][] newMat = new float[numRows][numColumns];
      for (int i = 0; i < numRows; i++) {
        System.arraycopy(matrix[i], 0, newMat[i], 0, numColumns);
      }    
      return newMat;
    }

    public float det () {
      if (numRows != numColumns) {
        throw new IllegalArgumentException("Matrix must be square.");
      }
      double d = (double) pivsign;
      for (int j = 0; j < numColumns; j++) {
        d *= matrix[j][j];
      }
      return (float)d;
    }

    public boolean isNonsingular () {
      for (int j = 0; j < numColumns; j++) {
        if (matrix[j][j] == 0)
          return false;
      }
      return true;
    }

    public float[] populate( float[] fullDataset, int[] indices) {
      float[] outputDat = new float[indices.length];
      for (int i = 0; i < indices.length; i++) {
        outputDat[i] = fullDataset[indices[i]];
      }
      return outputDat;
    }

    public double[] solve (float[] b, boolean returnZeroesForSingularCase) {
      if (b.length != numRows) {
        throw new IllegalArgumentException("Matrix row dimensions must agree.");
      }
      if (!this.isNonsingular()) {
        if (returnZeroesForSingularCase) return new double[numColumns];
        else throw new RuntimeException("Matrix is singular.");
      }

      // Copy right hand side with pivoting
      double[] X = floatToDouble(populate(b, piv));
      //System.out.println(X.length+","+b.length+","+LU[0].length);
      // Solve L*Y = B(piv,:)
      for (int k = 0; k < numColumns; k++) {
        for (int i = k+1; i < numColumns; i++) {
          X[i] -= X[k] * matrix[i][k];
        }
      }
      // Solve U*X = Y;
      for (int k = numColumns-1; k >= 0; k--) {
        X[k] /= matrix[k][k];
        for (int i = 0; i < k; i++) {
          X[i] -= X[k] * matrix[i][k];
        }
      }
      return X;
    }
  }
}
