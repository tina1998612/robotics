import controlP5.*;
import java.util.*;
import processing.serial.*;

ControlP5 cp5;
Serial myPort;

PFont f1;
color darkgray = #333333;
int inputInt;
double []data_from_car = new double[20];

int globalWidth = 80;
int globalHeight = 120;
int cnt;
int image_data, sys_time_dif=0;

int pixelSide = 5;
Boolean[][] pixelArray = new Boolean[globalHeight][globalWidth];

int background_color = 17, autoOrNot;
color red = #ff0000;
color black = #000000;

int arrayPosX, arrayPosY;
Boolean stop = false;
char data_received = ' ';
char keyPress = ' ';

String center_line_received_x = " ";
String data_string = "";
int total_var = 0;
boolean updateMenu, in_pid_graph = false;
long now;
int []center_x = new int[100];
int []center_y = new int[100];
int []sideL = new int[100];
int []sideR = new int[100];
float []error = new float[400]; 
float []last_error = new float[400];

PrintWriter output;

void setup() {
  
  frameRate(650);
  printArray(Serial.list());
  myPort = new Serial(this, Serial.list()[1], 115200);
  myPort.buffer(1);
  size(800, 700 ,P3D);
  f1 = createFont("Helvetica", 12);
  
  background( 220 );

  cp5 = new ControlP5( this );
  cp5.addButton("auto");
  cp5.addButton("manual");
  cp5.addButton("stop_").setLabel("stop"); //because "stop" is a conserved word
  cp5.addButton("clear");
  cp5.addButton("PID_graph");
  
  Controller auto_btn = cp5.getController("auto");
  auto_btn.setColorBackground(color(#00AA00));
  auto_btn.setColorForeground(color(#00EE00));
  auto_btn.setPosition(715, 20);
  
  Controller manual_btn = cp5.getController("manual");
  manual_btn.setPosition(715, 50);
  
  Controller stop_btn = cp5.getController("stop_");
  stop_btn.setPosition(715, 80);
  stop_btn.setColorBackground(color(#AA0000));
  stop_btn.setColorForeground(color(#EE0000));
  
  Controller clear_btn = cp5.getController("clear");
  clear_btn.setPosition(715, 110);
  clear_btn.setColorBackground(color(#AAAA00));
  
  Controller pid_btn = cp5.getController("PID_graph");
  pid_btn.setPosition(715, 170);
  pid_btn.setColorBackground(color(#880088));
  pid_btn.setColorForeground(color(#CC00CC));

  // create a custom SilderList with name menu, notice that function 
  // menu will be called when a menu item has been clicked.

  SilderList m = new SilderList( cp5, "menu", 250, 350 );

  String lines[] = loadStrings("list.txt"); // read data from a txt file(use variables saved before)
  total_var = lines.length;
  m.setPosition(40, 20);
  // add some items to our SilderList
  m.addItem(makeItem("KP", Float.valueOf(lines[0]), 0, 5));
  m.addItem(makeItem("KD", Float.valueOf(lines[1]), 0, 5));
  m.addItem(makeItem("KP_curve", Float.valueOf(lines[2]), 0, 10));
  m.addItem(makeItem("KD_curve", Float.valueOf(lines[3]), 0, 10));
  m.addItem(makeItem("speed", Float.valueOf(lines[4]), 0, 90));
  m.addItem(makeItem("sp", Float.valueOf(lines[4]), 0, 90));
  m.addItem(makeItem("sp", Float.valueOf(lines[4]), 0, 90));
}

public void auto(){
  println("auto btn is pressed!");
  myPort.write(0);
}
public void manual(){
  println("manual btn is pressed!");
  myPort.write(1);
}
public void stop_(){
  println("stop btn is pressed!");
  myPort.write(2);
}
public void clear(){
  println("clear btn is pressed!");
  myPort.clear();
}
public void PID_graph(){
  println("pi grapgh btn is pressed!");
  in_pid_graph ^= true;
}

// a convenience function to build a map that contains our key-value  
// pairs which we will then use to render each item of the SilderList.
//
Map<String, Object> makeItem(String theLabel, float theValue, float theMin, float theMax) {
  Map m = new HashMap<String, Object>();
  m.put("label", theLabel);
  m.put("sliderValue", theValue);
  m.put("sliderValueMin", theMin);
  m.put("sliderValueMax", theMax);
  return m;
}

void getImage(int data) {

  String binData = "";
  int strLen = 0;
  
  binData = Integer.toBinaryString(data);
  strLen = binData.length();
  //println(binData);
  
  for (int i=0; i<8; i++) {
    
    if (i < strLen) {
      
      if (binData.charAt(strLen-1-i) == '1') {
        pixelArray[arrayPosY][arrayPosX+7-i] = true; 
      } else {
        pixelArray[arrayPosY][arrayPosX+7-i] = false;
      }
      
    } else {
      pixelArray[arrayPosY][arrayPosX+7-i] = false;
    }
    
  }
  
  arrayPosX += 8;
  
  if (arrayPosX >= globalWidth) {
    arrayPosX = 0;
    arrayPosY++;
  }
  if (arrayPosY >= globalHeight) {
    arrayPosY = 0;
  }

}


void outputImage() {
  
  String tmp = "";
 
  for (int y=0; y<globalHeight; y++) {
    for (int x=0; x<globalWidth; x++) {
      if (pixelArray[y][x]) {
        //print("1");
        tmp+="1";
        fill(black);
      } else {
        tmp+="0";
        fill(255);
      }
      noStroke();
      rect(pixelSide*x, pixelSide*y, pixelSide, pixelSide);
    }
    if(y%5==0) {
      fill(color(#dddd00));
      text(y, 0, y*5);
    }
  }
  //println(tmp);
  output = createWriter("hi.txt");
  output.println(tmp);
  output.close();
  for(int i=0;i<data_from_car[15];i++){
    fill(red);
    noStroke();
    rect(pixelSide*center_x[i], pixelSide*center_y[i], pixelSide, pixelSide);
  }
  
}
void keyPressed() {
  
  keyPress = key;
  if(keyPress == ' ') {
    myPort.write(' ');
    stop = true;
  }
  else {
    stop = false;
  }
}
void keyReleased(){
  if(keyPress != '.' && keyPress!= ','){
    myPort.write(2);
    stop = true;
  }  
}


void menu(int i) {
  //println("got some slider-list event from item with index "+i);
}

public void controlEvent(ControlEvent theEvent) {
  if (theEvent.isFrom("menu")) {
    int index = int(theEvent.getValue());
    //println("got a slider event from item : "+m);
    data_string = "";
    output = createWriter("list.txt"); //save changed variables in to txt file
    for(int i=0;i<total_var;i++){
      String temp = ((SilderList)theEvent.getController()).getItem(i).get("sliderValue")+" ";
      int temp2 = floor(Float.valueOf(temp)*100);

      output.println(temp); 
      //data_string += Integer.toString(temp2)+" ";
      
      myPort.write(Integer.toString(temp2));
      myPort.write('f');
    }
     myPort.write('\n');
     output.close(); // Finishes the file
     //println(data_string);
  }
}

void draw() {
 
  pushMatrix();
  translate(300, 630);
  noStroke();
  fill(220);
  rect(-5, -9, 450, 60);
  fill(0);
  text("key pressed: "+keyPress, 0, 0); 
  text("received: "+Character.toString(data_received), 0, 15);
  //text("center_line_x: "+(center[45]+center[46]+center[47]+center[48]+center[49]+center[50])/6, 0, 30);
  pushMatrix();
  translate(60, 0);
  text("Lencoder_count: "+String.valueOf(data_from_car[9]), 80, 0);
  text("Rencoder_count: "+String.valueOf(data_from_car[10]), 80, 15);
  text("algo_execute_time: "+String.valueOf(sys_time_dif)+"ms", 80, 30);
  text("in_turn: "+String.valueOf(data_from_car[11]), 250, 0);
  text("servo_deg: "+String.valueOf(data_from_car[13]), 250, 15);
  popMatrix();
  popMatrix();

  if(myPort.available() > 0){
    inputInt = myPort.read();
    if(inputInt!=170) println(inputInt);
    if(inputInt==170){
      cnt = 0;
        arrayPosX = 0;
        arrayPosY = 0;
        now = System.currentTimeMillis();
        while (cnt<globalWidth*globalHeight/8) {
          if (myPort.available() > 0) {  
            now = System.currentTimeMillis();
            image_data = myPort.read();
            getImage(image_data);
            cnt++;
          }
          if(System.currentTimeMillis()-now>3000){ //if it lags too long -> break (otherwise it will stock in this while loop)
            break;
          }
        }
        noStroke();
        pushMatrix();
        translate(300, 20);
        if(in_pid_graph){
          fill(255);
          rect(0, 0, 400, 300);
          pushMatrix();
          translate(0, 150);
          stroke(red);
          line(0, 0, 400, 0);
          stroke(color(#000033));
          //error[0] = (center[50]-40)*3.5;
          last_error[0] = error[0];
          for(int i=1;i<400;i++){
            line(i-1, -error[i-1], i, -error[i]); // positive should be on the top of the red line
            last_error[i] = error[i];
            error[i] = last_error[i-1];
          }
          popMatrix();
        }
        else outputImage();
        popMatrix();
        cnt = 0;
        now = System.currentTimeMillis();
        while(cnt<4){
          if(myPort.available()>0) {  
            data_from_car[cnt] = myPort.read();
            data_from_car[cnt]/=10;
            println(data_from_car[cnt]);
            cnt++;
          }
          if(System.currentTimeMillis()-now>3000){
            break;
          }
        }
    }
    /*switch(inputInt){
      case 170:
        
        
        
        
      case 171:
        cnt = 0;
        while(cnt<6){
          if(myPort.available()>0) {
            data_from_car[cnt] = myPort.read();
            switch(cnt){
              case 0:
                data_from_car[cnt]/=100; // since Byte can only pass integers, we multiply 100 for sending data and divide 100 once received
                break;
              case 1:
                data_from_car[cnt]/=100;
                break;
              case 2:
                data_from_car[cnt]/=100;
                break;
              case 3:
                data_from_car[cnt]/=100;
                break;
              case 4:
                data_from_car[cnt]/=100;
                break;
            }
            cnt++;
            now = System.currentTimeMillis();
          }
          if(System.currentTimeMillis()-now>3000){
            break;
          }
        }
        //println("**********data from car*************");
        
        cnt = 0;
        while(cnt<data_from_car[15]){
          if(myPort.available()>0){
            center_x[cnt] = myPort.read();
            cnt++;
            now = System.currentTimeMillis();
          }
          if(System.currentTimeMillis()-now>3000){
            break;
          }
        }
        
        cnt=0;
        while(cnt<data_from_car[cnt]){
          if(myPort.available()>0){
            center_y[cnt] = myPort.read();
            cnt++;
            now = System.currentTimeMillis();
          }
          if(System.currentTimeMillis()-now>3000){
            break;
          }
        }
     
        // draw mode on the top
        fill(220);
          rect(400, 0, 300, 12);
          fill(0);
          if(data_from_car[5]==1){ // if in auto
            text("Auto Mode", 450, 12);
          } else{
            text("Manual Mode", 450, 12);
          }
        break;
       case 169:
         if(myPort.available()>0){
            data_received = myPort.readChar();
         }
         break;
         
      }*/
    }
}


// A custom Controller that implements a scrollable SilderList.  
// Here the controller uses a PGraphics element to render customizable 
// list items. The SilderList can be scrolled using the scroll-wheel,  
// touchpad, or mouse-drag. Slider are triggered by a press or drag.  
// clicking the scrollbar to the right makes the list scroll to the item  
// correspoinding to the click-location.  
 
class SilderList extends Controller<SilderList> {

  float pos, npos;
  int itemHeight = 60;
  int scrollerLength = 40;
  int sliderWidth = 150;
  int sliderHeight = 15;
  int sliderX = 10;
  int sliderY = 25;

  int dragMode = 0;
  int dragIndex = -1;

  List< Map<String, Object>> items = new ArrayList< Map<String, Object>>();
  PGraphics menu;

  SilderList(ControlP5 c, String theName, int theWidth, int theHeight) {
    super( c, theName, 0, 0, theWidth, theHeight );
    c.register( this );
    menu = createGraphics(getWidth(), getHeight());

    setView(new ControllerView<SilderList>() {

      public void display(PGraphics pg, SilderList t ) {
        if (updateMenu) {
          updateMenu();
        }
        if (inside() ) { // draw scrollbar
          menu.beginDraw();
          int len = -(itemHeight * items.size()) + getHeight();
          int ty = int(map(pos, len, 0, getHeight() - scrollerLength - 2, 2 ) );
          menu.fill( 128 );
          menu.rect(getWidth()-6, ty, 4, scrollerLength );
          menu.endDraw();
        }
        pg.image(menu, 0, 0);
      }
    }
    );
    updateMenu();
  }

  // only update the image buffer when necessary - to save some resources
  void updateMenu() {
    int len = -(itemHeight * items.size()) + getHeight();
    npos = constrain(npos, len, 0);
    pos += (npos - pos) * 0.1;

    /// draw the SliderList
    menu.beginDraw();
    menu.noStroke();
    menu.background(240);
    menu.textFont(cp5.getFont().getFont());
    menu.textSize(12);
    menu.pushMatrix();
    menu.translate( 0, int(pos) );
    menu.pushMatrix();

    int i0 = PApplet.max( 0, int(map(-pos, 0, itemHeight * items.size(), 0, items.size())));
    int range = ceil((float(getHeight())/float(itemHeight))+1);
    int i1 = PApplet.min( items.size(), i0 + range );

    menu.translate(0, i0*itemHeight);

    for (int i=i0;i<i1;i++) {
      Map m = items.get(i);
      menu.noStroke();
      menu.fill(200);
      menu.rect(0, itemHeight-1, getWidth(), 1 );
      menu.fill(darkgray);
      // uncomment the following line to use a different font than the default controlP5 font
      //menu.textFont(f1); 
      String txt = String.format("%s   %.2f -- %.2f", m.get("label").toString().toUpperCase(), f(items.get(i).get("sliderValue")), data_from_car[i]);
      menu.text(txt, 10, 20);
      menu.fill(255);
      menu.rect(sliderX, sliderY, sliderWidth, sliderHeight);
      menu.fill(100, 230, 128);
      float min = f(items.get(i).get("sliderValueMin"));
      float max = f(items.get(i).get("sliderValueMax"));
      float val = f(items.get(i).get("sliderValue"));
      //println(items.get(i).get("sliderValue"));
      //println("****************");
      menu.rect(sliderX, sliderY, map(val, min, max, 0, sliderWidth), sliderHeight);
      menu.translate( 0, itemHeight );
    }
    menu.popMatrix();
    menu.popMatrix();
    menu.endDraw();
    updateMenu = abs(npos-pos)>0.01 ? true:false;
  }

  // when detecting a click, check if the click happend to the far right,  
  // if yes, scroll to that position, otherwise do whatever this item of 
  // the list is supposed to do.
  public void onClick() {
    if (getPointer().x()>getWidth()-10) {
      npos= -map(getPointer().y(), 0, getHeight(), 0, items.size()*itemHeight);
      updateMenu = true;
    }
  }


  public void onPress() {
    int x = getPointer().x();
    int y = (int)(getPointer().y()-pos)%itemHeight;
    boolean withinSlider = within(x, y, sliderX, sliderY, sliderWidth, sliderHeight); 
    dragMode =  withinSlider ? 2:1;
    if (dragMode==2) {
      dragIndex = getIndex();
      float min = f(items.get(dragIndex).get("sliderValueMin"));
      float max = f(items.get(dragIndex).get("sliderValueMax"));
      float val = constrain(map(getPointer().x()-sliderX, 0, sliderWidth, min, max), min, max);
      items.get(dragIndex).put("sliderValue", val);
      setValue(dragIndex);
    }
    updateMenu = true;
  }

  public void onDrag() {
    switch(dragMode) {
      case(1): // drag and scroll the list
      npos += getPointer().dy() * 2;
      updateMenu = true;
      break;
      case(2): // drag slider
      float min = f(items.get(dragIndex).get("sliderValueMin"));
      float max = f(items.get(dragIndex).get("sliderValueMax"));
      float val = constrain(map(getPointer().x()-sliderX, 0, sliderWidth, min, max), min, max);
      items.get(dragIndex).put("sliderValue", val);
      setValue(dragIndex);
      updateMenu = true;
      break;
    }
  } 

  public void onScroll(int n) {
    npos += ( n * 4 );
    updateMenu = true;
  }
  void addItem(Map<String, Object> m) {
    items.add(m);
    updateMenu = true;
  }

  Map<String, Object> getItem(int theIndex) {
    return items.get(theIndex);
  }

  private int getIndex() {
    int len = itemHeight * items.size();
    int index = int( map( getPointer().y() - pos, 0, len, 0, items.size() ) ) ;
    return index;
  }
}

public static float f( Object o ) {
  return ( o instanceof Number ) ? ( ( Number ) o ).floatValue( ) : Float.MIN_VALUE;
}

public static boolean within(int theX, int theY, int theX1, int theY1, int theW1, int theH1) {
  return (theX>theX1 && theX<theX1+theW1 && theY>theY1 && theY<theY1+theH1);
}
