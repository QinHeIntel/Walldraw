//沁和智能科技  www.qhebot.com

#include <TinyStepper_28BYJ_48.h>		//步进电机的库 如果没有该lib请按Ctrl+Shift+I 从 库管理器中搜索 Stepper_28BYJ_48，并安装
#include <Servo.h>
#include <SD.h>  //需要SD卡读卡器模块，或者tf读卡器模块 如果没有该lib请按Ctrl+Shift+I 从 库管理器中搜索 SD，并安装
#include "FastLED.h"

#define NUM_LEDS 6    //LED数目
#define DATA_PIN A3     //绿色数据线接在几号端口

long int TATOL_LINE = 0;
long int curline=0;
CRGB leds[NUM_LEDS];


//调试代码标志，去掉注释，可以输出调试信息（程序运行会慢）
//#define VERBOSE         (1)
//调试标志


#define STEPS_PER_TURN  (2048)  //步进电机一周步长 2048步转360度
#define SPOOL_DIAMETER  (35)    //线轴直径mm
#define SPOOL_CIRC      (SPOOL_DIAMETER * 3.1416)  //线轴周长 35*3.14=109.956
#define TPS             (SPOOL_CIRC / STEPS_PER_TURN)  //步进电机步距，最小分辨率 每步线绳被拉动的距离  0.053689mm

#define step_delay      1   //步进电机每步的等候时间 （微妙）
#define TPD             300   //转弯等待时间（毫秒），由于惯性笔会继续运动，暂定等待笔静止再运动。

//两个电机的旋转方向  1正转  -1反转  
//调节进出方向可垂直反转图像
#define M1_REEL_OUT     1     //放出线
#define M1_REEL_IN      -1      //卷入线
#define M2_REEL_OUT     -1      //放出线
#define M2_REEL_IN      1     //卷入线

static long laststep1, laststep2; //当前线长度 记录笔位置

#define X_SEPARATION  1025           //两绳上方的水平距离mm 
#define LIMXMAX       ( X_SEPARATION*0.5)   //x轴最大值  0位在画板中心
#define LIMXMIN       (-X_SEPARATION*0.5)   //x轴最小值

/* 垂直距离的参数： 正值在画板下放，理论上只要画板够大可以无限大，负值区域在笔（开机前）的上方 
详细介绍见说明文档 https://github.com/shihaipeng03/Walldraw
*/
#define LIMYMAX         (-600)   //y轴最大值 画板最下方
#define LIMYMIN         (600)    //y轴最小值 画板最上方  左右两线的固定点到笔的垂直距离，尽量测量摆放准确，误差过大会有畸变
                //值缩小画图变瘦长，值加大画图变矮胖 

//抬笔舵机的角度参数  具体数值要看摆臂的安放位置，需要调节
#define PEN_UP_ANGLE    110  //抬笔 
#define PEN_DOWN_ANGLE  20  //落笔
//需要调节的参数 =============================================

#define PEN_DOWN 1  //笔状态  下笔
#define PEN_UP 0    //笔状态  抬笔

// plotter position 笔位置.
static float posx;
static float posy;
static float posz;  // pen state

// pen state 笔状态（抬笔，落笔）.
static int ps;

/*以下为G代码通讯参数 */
#define BAUD            (115200)    //串口速率，用于传输G代码或调试 可选9600，57600，115200 或其他常用速率

// Serial comm reception
static int sofar;               // Serial buffer progress

static float mode_scale;   //比例

File myFile;

Servo pen;

TinyStepper_28BYJ_48 m1; //(7,8,9,10);  //M1 L步进电机   in1~4端口对应UNO  7 8 9 10
TinyStepper_28BYJ_48 m2; //(2,3,5,6);  //M2 R步进电机   in1~4端口对应UNO 2 3 5 6

//------------------------------------------------------------------------------
//反向运动 - 将XY坐标转换为长度L1，L2 
void IK(float x,float y,long &l1, long &l2) {
  float dy = y - LIMYMIN;
  float dx = x - LIMXMIN;
  l1 = round(sqrt(dx*dx+dy*dy) / TPS);
  dx = x - LIMXMAX;
  l2 = round(sqrt(dx*dx+dy*dy) / TPS);
}


void pen_down()
{
  if (ps==PEN_UP_ANGLE)
  {
    ps=PEN_DOWN_ANGLE;
    pen.write(ps);
    delay(TPD);
  }

}

void pen_up()
{
  if (ps==PEN_DOWN_ANGLE)
  {
    ps=PEN_UP_ANGLE;
    pen.write(ps);
  }

  
}


//------------------------------------------------------------------------------
// instantly move the virtual plotter position
// does not validate if the move is valid
static void teleport(float x, float y) {
  posx = x;
  posy = y;
  long l1,l2;
  IK(posx, posy, l1, l2);
  laststep1 = l1;
  laststep2 = l2;
}


//==========================================================
//参考————斜线程序
void moveto(float x,float y) {

  long l1,l2;
  IK(x,y,l1,l2);
  long d1 = l1 - laststep1;
  long d2 = l2 - laststep2;


  long ad1=abs(d1);
  long ad2=abs(d2);
  int dir1=d1>0 ? M1_REEL_IN : M1_REEL_OUT;
  int dir2=d2>0 ? M2_REEL_IN : M2_REEL_OUT;
  long over=0;
  long i;

  if(ad1>ad2) {
    for(i=0;i<ad1;++i) {
      
      m1.moveRelativeInSteps(dir1);
      over+=ad2;
      if(over>=ad1) {
        over-=ad1;
        m2.moveRelativeInSteps(dir2);
      }
      delayMicroseconds(step_delay);
     }
  } 
  else {
    for(i=0;i<ad2;++i) {
      m2.moveRelativeInSteps(dir2);
      over+=ad1;
      if(over>=ad2) {
        over-=ad2;
        m1.moveRelativeInSteps(dir1);
      }
      delayMicroseconds(step_delay);
    }
  }
  laststep1=l1;
  laststep2=l2;
  posx=x;
  posy=y;  
}

//------------------------------------------------------------------------------
//长距离移动会走圆弧轨迹，所以将长线切割成短线保持直线形态
static void line_safe(float x,float y) {
  // split up long lines to make them straighter?
  float dx=x-posx;
  float dy=y-posy;

  float len=sqrt(dx*dx+dy*dy);
  
  if(len<=TPS) {
    moveto(x,y);
    return;
  }
  
  // too long!
  long pieces=floor(len/TPS);
  float x0=posx;
  float y0=posy;
  float a;  
  for(long j=0;j<=pieces;++j) {
    progress_light_bar(curline*100/TATOL_LINE);
    a=(float)j/(float)pieces;

    moveto((x-x0)*a+x0,
         (y-y0)*a+y0);
  }
  moveto(x,y);
}

void line(float x,float y) {
  line_safe(x,y);
}

void nc(String st){

  String xx,yy,zz;
  int ok=1;
  st.toUpperCase();
  
  float x,y,z;
  int px,py,pz;
  px = st.indexOf('X');
  py = st.indexOf('Y');
  pz = st.indexOf('Z');
  if (px==-1 || py==-1) ok=0; 
  if (pz==-1){ pz=st.length();}
  else{   
      zz = st.substring(pz+1,st.length());
      z  = zz.toFloat();
      if (z>0)  pen_up();
      if (z<=0) pen_down();
  }

  xx = st.substring(px+1,py);
  yy = st.substring(py+1,pz);

  xx.trim();//缩进，去掉末尾空格*/
  yy.trim();

  if (ok) line(xx.toFloat(),yy.toFloat());
}

//**********************
void drawfile( String filename){
  String rd="";
  char rr=0;
  Serial.print("[");
  Serial.print(filename);
  myFile = SD.open(filename);
  
  if (myFile) {
    Serial.println("] Opened");
    Serial.println(myFile.read());
    Serial.println(myFile.read());
    Serial.println(myFile.read());
    while (myFile.available()) {
      rr=myFile.read();
      if (rr == char(10)){
          curline++;
          rd.toUpperCase();
          TATOL_LINE = atol( &(rd.substring(rd.indexOf('N')+1,rd.length() ))[0] );
//          TATOL_LINE = rd.substring(rd.indexOf('N')+1,rd.length() ).toInt();
          Serial.print(" TATOL_LINE  ");
          Serial.println(TATOL_LINE);
          rd="";
          break;
       }else {rd+=rr;}
       Serial.println(rd);
    }
    
    while (myFile.available()) {
      rr=myFile.read();
      
      if (rr == char(10)){
          curline++;
          Serial.print("Run nc #");
          Serial.print(curline);
          Serial.println(" : "+rd);
          nc(rd);
          rd="";
          progress_light_bar(curline*100/TATOL_LINE);
       }
       else {rd+=rr;}
    }
    Serial.println(myFile.read());
    Serial.println(myFile.read());
    Serial.println(myFile.read());
    Serial.println("CLOSED");
    myFile.close();
  }
  else
    Serial.println("Open file error.");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD);
  m1.connectToPins(7,8,9,10); //M1 L步进电机   in1~4端口对应UNO  7 8 9 10
  m2.connectToPins(2,3,5,6);  //M2 R步进电机   in1~4端口对应UNO 2 3 5 6
  m1.setSpeedInStepsPerSecond(10000);
  m1.setAccelerationInStepsPerSecondPerSecond(100000);
  m2.setSpeedInStepsPerSecond(10000);
  m2.setAccelerationInStepsPerSecondPerSecond(100000);

  //抬笔舵机
  pen.attach(A0);
  ps=PEN_UP_ANGLE;
  pen.write(ps);

  //将当前笔位置设置为0，0
  teleport(0, 0);
  //缩放比例
  mode_scale = 1;

  LEDS.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);  // 初始化光带 
  FastLED.setBrightness(15);                            // 设置光带亮度

  if (!SD.begin(4)) {
    Serial.println("initialization SD failed!");
    while (1);
  }
  Serial.println("Test OK!");
}

void loop() { 
  drawfile("1.nc");  //1.nc 是Gcode代码的文件名 ，需要将g代码保存在sd卡上。
  
  //苹果系统请把文件名改成 1.txt 之类的，在复制nc文件的时候，系统可能会改变文件名
  while(1);
}

long int BLIKT = 0;
void progress_light_bar(byte progress){
  for( byte i = 0; i < progress/17; i++ ){
    leds[i].b = 255;
  }
  
  if( millis() - BLIKT > 600 ){ leds[progress/17].b = ~leds[progress/17].b; BLIKT = millis(); } 
  if(progress >= 99){leds[5].b = 255;}
  FastLED.show();
}
