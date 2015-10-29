//ref  http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//ref  http://forum.processing.org/one/topic/easy-to-use-quaternion-class.html

//I used Processing more than ver 3.0.

// you wait move sensor until finishing calibration.
// cubte texture image and real xyz is not same.
// if success, you can see look like https://www.youtube.com/watch?v=ecEN49Y1rZ8

import processing.serial.*;

//global
Serial g_Port; 
String g_SerialLine = "";
Quaternion g_NowQ = new Quaternion();
float g_LastUpdate = 0;
CubeObj g_Cube;

float g_Roll = 0, g_Pitch = 0, g_Yaw = 0;

void setup()
{
  String arduinoPort = Serial.list()[0];             
  g_Port = new Serial(this, arduinoPort, 9600);
  size(640, 360, P3D);
  g_Cube = new CubeObj();
}

void draw()
{
  background(255);
  noStroke();
  translate(width/2.0, height/2.0, -100); 
  scale(100);

  //Match processing Rotate
  rotateX(g_Roll);
  rotateZ(g_Pitch); 
  rotateY(g_Yaw);
  g_Cube.Draw();
}

void serialEvent(Serial p)
{

  if ( g_Port.available() > 0 ) 
  {
    int recv_data = g_Port.read();
    char c = char(recv_data);
    g_SerialLine += c; 
    if (c == '\n')
    {
      print(g_SerialLine);
      if (ValidateSerialLine(g_SerialLine))
      {
        SerialData data = GetSerialData(g_SerialLine);
        if (!data.CheckNaN()) return;
        
        //ref http://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf
        float sampleFreq = 512f; //rate of convergence to remove gyroscope measurement errors
        float beta = 0.1f; //zero gyroscope measurement erros 
        Quaternion updateQ = MadgwickAHRS(sampleFreq, beta, g_NowQ, data);

        if (updateQ == null) return;

        //Program rotate and real roate are reverse
        Quaternion conj = updateQ.conjugate();

        boolean ok = SetEulerAngles(conj);

        if (ok)  
        {
          float now = millis();
          float span = now - g_LastUpdate;
          g_NowQ = updateQ;
          g_NowQ.Output();
          println("TimeSpan " + span +  " Roll " + g_Roll + " Pitch " + g_Pitch + " Yaw " + g_Yaw);
          println();
          g_LastUpdate = now;
        }
      }

      g_SerialLine = "";
    }
  }
}




boolean SetEulerAngles(Quaternion q)
{
  float w = q.W, x = q.X, y= q.Y, z = q.Z;

  float yy = atan2(2 * y * w - 2 * x * z, 1 - 2 * y * y - 2 * z * z);
  float pp = asin(2 * x * y + 2 * z * w);
  float rr = atan2(2 * x * w - 2 * y * z, 1 - 2 * x * x - 2 * z * z );

  if (x * y + z * w == 0.5) 
  {
    yy = 2 * atan2(x, w);
    pp = 0;
  }

  if (x * y + z * w == -0.5)
  {
    yy = - 2 * atan2(x, w); 
    pp = 0;
  }

  boolean ok = false;
  if (!Float.isNaN(rr) && !Float.isNaN(pp) && !Float.isNaN(yy))
  {
    g_Roll = rr;
    g_Pitch = pp;
    g_Yaw = yy;
    ok = true;
  } else println("NULL");
  return ok;
}




//ref  http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
Quaternion MadgwickAHRS(float sampleFreq, float beta, Quaternion q, SerialData sd)
{
  float gx = sd.GX, gy = sd.GY, gz = sd.GZ; 
  float ax = sd.AX, ay = sd.AY, az = sd.AZ;
  float mx = sd.MX, my = sd.MY, mz = sd.MZ;
  float q0 = q.W, q1 = q.X, q2 = q.Y, q3 = q.Z;

  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) 
  {
    return  MadgwickAHRSupdateIMU(sampleFreq, beta, gx, gy, gz, ax, ay, az, q);
  }

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;   

    // Normalise magnetometer measurement
    recipNorm = 1.0f / sqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * (1.0f / sampleFreq);
  q1 += qDot2 * (1.0f / sampleFreq);
  q2 += qDot3 * (1.0f / sampleFreq);
  q3 += qDot4 * (1.0f / sampleFreq);

  // Normalise quaternion
  recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  Quaternion result = new Quaternion(q0, q1, q2, q3);
  return result;
}

Quaternion MadgwickAHRSupdateIMU(float sampleFreq, float beta, float gx, float gy, float gz, float ax, float ay, float az, Quaternion q) 
{

  float q0 = q.W, q1 = q.X, q2 = q.Y, q3 = q.Z;

  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;   

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * (1.0f / sampleFreq);
  q1 += qDot2 * (1.0f / sampleFreq);
  q2 += qDot3 * (1.0f / sampleFreq);
  q3 += qDot4 * (1.0f / sampleFreq);

  // Normalise quaternion
  recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  return new Quaternion(q0, q1, q2, q3);
}






boolean ValidateSerialLine(String line)
{
  boolean ok = true;
  String[] sp = splitTokens(line, "_");
  if (sp.length != 3)  return false;
  //print(sp.length);
  if (line.startsWith("A:") != true) return false;

  String[] ass = splitTokens(sp[0], ":");
  if (ass.length != 2) return false;
  String[] as =  splitTokens(ass[1], ",");  
  if (as.length != 3) return false;

  String[] mss = splitTokens(sp[1], ":");
  if (mss.length != 2) return false;
  String[] ms =  splitTokens(mss[1], ","); 
  if (ms.length != 3) return false;

  String[] gss = splitTokens(sp[2], ":");
  if (gss.length != 2) return false;
  String[] gs =  splitTokens(gss[1], ",");  
  if (gs.length != 3) return false;

  return ok;
}

SerialData GetSerialData(String line)
{
  String[] sp = splitTokens(line, "_"); 

  String[] ass = splitTokens(sp[0], ":");
  String[] as =  splitTokens(ass[1], ",");

  String[] mss = splitTokens(sp[1], ":");
  String[] ms =  splitTokens(mss[1], ",");

  String[] gss = splitTokens(sp[2], ":");
  String[] gs =  splitTokens(gss[1], ",");

  return new SerialData(as, ms, gs);
}




public class SerialData 
{
  public float GX, GY, GZ; //rad/s
  public float AX, AY, AZ;
  public float MX, MY, MZ;
  public FloatList AllData = new FloatList();

  public SerialData(String[] _as, String[] _ms, String[] _gs)
  {
    AX = float(_as[0]); 
    AY = float(_as[1]); 
    AZ =  float(_as[2]); 

    MX = float(_ms[0]); 
    MY = float(_ms[1]); 
    MZ = float(_ms[2]); 

    GX = float(_gs[0]); 
    GY = float(_gs[1]); 
    GZ = float(_gs[2]);

    AllData.append(AX);
    AllData.append(AY);
    AllData.append(AZ);
    AllData.append(MX);
    AllData.append(MY);
    AllData.append(MZ);
    AllData.append(GX);
    AllData.append(GY);
    AllData.append(GZ);
  }

  public boolean CheckNaN()
  {
    boolean ok = true;
    for (int i = 0; i < AllData.size(); i++)
    {
      if (Float.isNaN(AllData.get(i))) 
      {
        ok = false;
        break;
      }
    }

    return ok;
  }

  public void Output()
  {
    println(
      "A:" + str(AX) + "," + str(AY) + "," + str(AZ) + "_" + 
      "M:" + str(MX) + "," + str(MY) + "," + str(MZ) + "_" +
      "G:" + str(GX) + "," + str(GY) + "," + str(GZ) 
      );
  }
}


//ref http://forum.processing.org/one/topic/easy-to-use-quaternion-class.html
public class Quaternion 
{
  public  float W, X, Y, Z;      // components of a quaternion

  // default constructor
  public Quaternion() 
  {
    W = 1.0;
    X = 0.0;
    Y = 0.0;
    Z = 0.0;
  }

  // initialized constructor
  public Quaternion(float w, float x, float y, float z) 
  {
    W = w;
    X = x;
    Y = y;
    Z = z;
  }

  public Quaternion conjugate () 
  {
    return new Quaternion(W, -X, -Y, -Z);
  }

  public boolean CheckNorm()
  {
    float norm = sqrt(X * X + Y * Y + Z * Z + W * W);
    if (norm == 0) return false;
    else return true;
  }

  public void Output()
  {
    println("W:" + W + " X:" + X + " Y:" + Y + " Z:" + Z);
  }
}


public class CubeObj
{
  PImage[] textures = new PImage[6];

  public CubeObj()
  {
    textureMode(NORMAL);
    textures[0] = loadImage("img/Front.png");
    textures[1] = loadImage("img/Back.png");
    textures[2] = loadImage("img/Bottom.png");
    textures[3] = loadImage("img/Top.png");
    textures[4] = loadImage("img/Right.png");
    textures[5] = loadImage("img/Left.png");
  }

  public void Draw()
  {

    beginShape(QUADS);
    texture(textures[0]);

    // +Z "front" face
    vertex(-1, -1, 1, 0, 0);
    vertex( 1, -1, 1, 1, 0);
    vertex( 1, 1, 1, 1, 1);
    vertex(-1, 1, 1, 0, 1);
    endShape();

    beginShape(QUADS);
    texture(textures[1]);
    // -Z "back" face
    vertex( 1, -1, -1, 0, 0);
    vertex(-1, -1, -1, 1, 0);
    vertex(-1, 1, -1, 1, 1);
    vertex( 1, 1, -1, 0, 1);
    endShape();

    beginShape(QUADS);
    texture(textures[2]);
    // +Y "bottom" face
    vertex(-1, 1, 1, 0, 0);
    vertex( 1, 1, 1, 1, 0);
    vertex( 1, 1, -1, 1, 1);
    vertex(-1, 1, -1, 0, 1);
    endShape();

    beginShape(QUADS);
    texture(textures[3]);
    // -Y "top" face
    vertex(-1, -1, -1, 0, 0);
    vertex( 1, -1, -1, 1, 0);
    vertex( 1, -1, 1, 1, 1);
    vertex(-1, -1, 1, 0, 1);
    endShape();

    beginShape(QUADS);
    texture(textures[4]);
    // +X "right" face
    vertex( 1, -1, 1, 0, 0);
    vertex( 1, -1, -1, 1, 0);
    vertex( 1, 1, -1, 1, 1);
    vertex( 1, 1, 1, 0, 1);
    endShape();

    beginShape(QUADS);
    texture(textures[5]);
    // -X "left" face
    vertex(-1, -1, -1, 0, 0);
    vertex(-1, -1, 1, 1, 0);
    vertex(-1, 1, 1, 1, 1);
    vertex(-1, 1, -1, 0, 1);

    endShape();
  }
}