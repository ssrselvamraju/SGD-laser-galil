#include <stdio.h>
#include <stdlib.h>
#include <ctime>
#include <time.h>
#include <string>
#include <cstring> 
#include <iomanip>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <sys/types.h>
#include <stdint.h>
#include <inttypes.h>
#include <ostream>
#include <netdb.h> 
#include <math.h>
#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>
#include <osgText/Text>
#include <osg/Geode>


using namespace std;

#define DRAW_ORIGIN    1
#define DRAW_OUTLINE    1
#define DRAW_RAYS       1
#define DRAW_POINTS     0
#define DRAW_DIFF       0

#define NUM_VERTS       1442

//#include <modbus.h>
#define IP_ADDRESS "10.0.0.100"
#define PORT 1502
#define SAFE_SPEED_REGISTER 14
#define OBSTRUCTION_ANGLE_REGISTER 15
#define BRAKE_COIL 10
//modbus_t *ctx;

osgViewer::Viewer viewer;
osg::Vec3Array* stopVertices = new osg::Vec3Array(NUM_VERTS);
osg::Vec3Array* brakeVertices = new osg::Vec3Array(NUM_VERTS);
osg::Vec3Array* slowVertices = new osg::Vec3Array(NUM_VERTS);
osg::Vec3Array* outlineVertices = new osg::Vec3Array(NUM_VERTS);
osg::Vec3Array* originVertices = new osg::Vec3Array(NUM_VERTS);
osg::Vec3Array* recorder_vertices = new osg::Vec3Array(NUM_VERTS);
osg::Vec3Array* diff_vertices = new osg::Vec3Array(NUM_VERTS);

osg::ref_ptr<osg::Group> root (new osg::Group);
osg::ref_ptr<osg::Geode> myshapegeode (new osg::Geode);
osg::ref_ptr<osg::Capsule> myCapsule (new osg::Capsule(osg::Vec3f(),1,2));
osg::ref_ptr<osg::ShapeDrawable> capsuledrawable (new osg::ShapeDrawable(myCapsule.get()));
osgText::Text* myText = new osgText::Text();

inline double deg2rad(const double val) { return val*0.0174532925199432957692369076848861;}


float sigmoid(float x)
{
     float exp_value;
     float return_value;

     /*** Exponential calculation ***/
     exp_value = exp((double) -x);

     /*** Final sigmoid value ***/
     return_value = 1 / (1 + exp_value);

     return return_value;
}


int fd_serialport;
int byteCounter = 0;
unsigned char inputBuffer[255];
unsigned int scanData[542];
char scanNumber[256];
int totalBytesRead = 0;

int    status;
struct addrinfo host_info;       
struct addrinfo *host_info_list; 
int    socketfd ;

int read(int bytes)
{
    int bytesRead = 0;
    bytesRead = read( fd_serialport, &inputBuffer[0], bytes);
    //printf ("%2d bytesRead: ", bytesRead);
    /*
    for (int i=0; i<bytesRead; i++){
        //printf(" %.2X ", (unsigned int) uc);
        printf(" %.2X ", inputBuffer[i]);
        if (inputBuffer[i] == '\0') {
            printf("\n");
        }
    }
    */
    //printf ("\n");
    if ((bytes == 2) && (bytesRead == 1)) {
        //usleep(100);
        bytesRead += read( fd_serialport, &inputBuffer[1], 1);
    } 

    if (bytesRead != bytes ) {
        printf ("Error, read requested %d bytes but read %d \n", bytes, bytesRead);
    }
    totalBytesRead += bytesRead;
    return bytesRead;
}

void initSerial()
{
    struct termios options;
    fd_serialport = open("/dev/LASER", O_RDWR | O_NOCTTY | O_NDELAY );
    //fd_serialport = open("/dev/ttyr00", O_RDWR | 0 | 0 );

    if(fd_serialport == -1){
        perror("Unable to open /dev/LASER");
    }

    tcgetattr(fd_serialport, &options);
    //int success = cfsetispeed(&options, B460800);
    int success = cfsetispeed(&options, B460800);
    printf ("ispeed success = %d\n", success);
    success = cfsetospeed(&options, B460800);
    printf ("ospeed success = %d\n", success);
   /* 
    options.c_cflag |= (CLOCAL | CREAD);    
    options.c_cflag |= PARENB;
    options.c_cflag |= PARODD;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_iflag |= (INPCK | ISTRIP);
    options.c_cc[VMIN] = 1;  //stop after one byte
    options.c_cc[VTIME] = 100;  //stop after 100ms timeout
*/
    tcsetattr(fd_serialport, TCSANOW, &options);
    fcntl(fd_serialport, F_SETFL, 0);

}

int foundHeader = 0;


string getResponseForMsg(string msg){
    send(socketfd, msg.c_str() , msg.length(), 0);
    char recvbuf[512];
    ssize_t bytes_received = recv(socketfd, recvbuf,512, 0);
    string receivedString;                        
    receivedString.assign(recvbuf,bytes_received); 
    char ch = *receivedString.rbegin();
    printf ("Last character is %c \n", ch);
    if (bytes_received == 0) {
        printf ("host shut down.\n");
    }
    if (bytes_received == -1)  {
        printf ("receive error!\n");
    }
    return receivedString;
}

void updateData() {
    //discard characters until we see four byte header = 00 00 00 00
#define DEBUG_UPDATE 0
    int zeroCounter = 0;
    int byteCounter = 0; 
    if (foundHeader == 0 )
        {
        while (foundHeader == 0)
        {
            int bytesRead = read(1);
            //printf("\nbytesRead: %d \n", bytesRead);
            for (int i=0; i<bytesRead; i++){
                byteCounter++;
                //printf(" %.2X ", inputBuffer[i]);
                if (inputBuffer[i] == '\0') {
                    zeroCounter++;
                } else {
                    zeroCounter = 0;
                }
                if (zeroCounter == 6) {
                    foundHeader = 1;
                    
                    if (foundHeader >= 1) {
                        printf("found initial header after (should be 1108 bytes): %d bytes ", byteCounter);
                        /*
                        if (byteCounter != 1108 ) {
                            printf(" [FAIL]\n" );
                        } else {
                            printf(" [OK]\n" );
                        }
                        */
                    }
                    
                    zeroCounter = 0;
                    byteCounter = 0;
                    if (DEBUG_UPDATE) printf("found header 00 00 00 00 \n");
                    break;
                }
            }
        }
    } else {
        int bytesRead = read(6);
        for (int i=0; i<bytesRead; i++){
            byteCounter++;
            //printf(" repeat check [%d] =  %.2X \n", i, inputBuffer[i]);
            if (inputBuffer[i] == '\0') {
                zeroCounter++;
            } else {
                zeroCounter = 0;
            }
            if (zeroCounter == 6) {
                foundHeader = 1;
                if (DEBUG_UPDATE) printf("found repeat header after (should be 6 bytes): %d bytes \n", byteCounter);
            } else {
                foundHeader = 0;
            }
        }
    }


    //check size of telegram should be 02 28
    read(2);
    if (DEBUG_UPDATE) printf("size of telegram (should be 02 28): %.2X %.2X ", inputBuffer[0], inputBuffer[1] );
    if (inputBuffer[0] != 0x02 || inputBuffer[1] != 0x28 ) {
        if (DEBUG_UPDATE) printf (" [FAIL]\n");
    } else {
        if (DEBUG_UPDATE) printf (" [OK]\n");
    }

    //check coordination flag and device code should be FF 07
    read(2);
    if (DEBUG_UPDATE) printf("coordination flag and device code (should be FF 07): %.2X %.2X ", inputBuffer[0], inputBuffer[1] );
    if (inputBuffer[0] != 0xFF || inputBuffer[1] != 0x07 ) {
        if (DEBUG_UPDATE) printf (" [FAIL]\n");
    } else {
        if (DEBUG_UPDATE) printf (" [OK]\n");
    }

    //check coordination flag and device code should be FF 07
    read(2);
    if (DEBUG_UPDATE) printf("protocol code (should be 02 01): %.2X %.2X ", inputBuffer[0], inputBuffer[1] );
    if (inputBuffer[0] != 0x02 || inputBuffer[1] != 0x01 ) {
        if (DEBUG_UPDATE) printf (" [FAIL]\n");
    } else {
        if (DEBUG_UPDATE) printf (" [OK]\n");
    }

    //check status normal 0x0000 or lockout 0x0001 
    read(2);
    if (DEBUG_UPDATE) printf("status code (should be 00 00): %.2X %.2X ", inputBuffer[0], inputBuffer[1] );
    if (inputBuffer[0] != 0x00 || inputBuffer[1] != 0x00 ) {
        if (DEBUG_UPDATE) printf (" [FAIL]\n");
        if (inputBuffer[1] == 0x01) {
            if (DEBUG_UPDATE) printf (" scanner is in lockout state  [FAIL]\n");
        }
    } else {
        if (DEBUG_UPDATE) printf (" [OK]\n");
    }

    //check scan number (timestamp)
    read(4);
    if (DEBUG_UPDATE) printf("scan number: 0x%.2X%.2X%.2X%.2X \n", inputBuffer[3], inputBuffer[2], inputBuffer[1], inputBuffer[0] );
    sprintf(scanNumber, "0x%.2X%.2X%.2X%.2X", inputBuffer[3], inputBuffer[2], inputBuffer[1], inputBuffer[0] );
    //printf("epoch: %d, scanNumber: %s, totalBytesRead: %d\n", (int)time(0), scanNumber, totalBytesRead);

    read(2);
    if (DEBUG_UPDATE) printf("telegram number: 0x%.2X%.2X \n", inputBuffer[1], inputBuffer[0] );
    char telegramNumber[256];
    sprintf(telegramNumber, "0x%.2X%.2X", inputBuffer[1], inputBuffer[0] );


    //check ID for measurement data should be BB BB
    read(2);
    if (DEBUG_UPDATE) printf("measurement ID (should be BB BB): %.2X %.2X ", inputBuffer[0], inputBuffer[1] );
    if (inputBuffer[0] != 0xBB || inputBuffer[1] != 0xBB ) {
        if (DEBUG_UPDATE) printf (" [FAIL]\n");
    } else {
        if (DEBUG_UPDATE) printf (" [OK]\n");
    }
    
    //check ID for measurement data should be 11 11
    read(2);
    if (DEBUG_UPDATE) printf("measured values for angular range 1 (should be 11 11): %.2X %.2X ", inputBuffer[0], inputBuffer[1] );
    if (inputBuffer[0] != 0x11 || inputBuffer[1] != 0x11 ) {
        if (DEBUG_UPDATE) printf (" [FAIL]\n");
    } else {
        if (DEBUG_UPDATE) printf (" [OK]\n");
    }

    //read angular measurements 1 thru 541
    for (int range=0; range < 541; range++) {
        read(2);
        if (DEBUG_UPDATE) printf("%s, %s, range, %d, 0x%.2X%.2X, ", scanNumber, telegramNumber, range, inputBuffer[1], inputBuffer[0] );
    
        //if (DEBUG_UPDATE) printf("actual unsigned chars: %.2X %.2X \n", inputBuffer[0], inputBuffer[1] );

        //if (DEBUG_UPDATE) printf("16bit endian swap: 0x%.2X%.2X \n", inputBuffer[1], inputBuffer[0] );

        unsigned int num = (inputBuffer[1] << 8) + inputBuffer[0];
        if (DEBUG_UPDATE) printf("16bit num, %u, ", num);

        bool bit15 = num & 0x8000;
        bool bit14 = num & 0x4000;
        bool bit13 = num & 0x2000;

        if (DEBUG_UPDATE) printf("bit15 fieldB, %d, ", bit15);
        if (DEBUG_UPDATE) printf("bit14 fieldA, %d, ", bit14);
        if (DEBUG_UPDATE) printf("bit13 glare, %d, ", bit13);

        unsigned int meas = num & 0x1FFF;
        if (DEBUG_UPDATE) printf("bits 12-0 distance in cm: %u\n", meas);
        scanData[range] = meas;
    
    }

    //check CRC
    read(2);
    if (DEBUG_UPDATE) printf("CRC 0x%.2X%.2X \n", inputBuffer[1], inputBuffer[0] );
}
    
int closest_y_cm; //max distance is 3000cm = 30meters = 30000mm
int stop;
double speed;
double tdist;

void updateVerts()
{
    //vertices = new osg::Vec3Array(NUM_VERTS);
    double angle=0.0;
    int px = 0, py = 0, pz = 0;
    stop = 541;
    closest_y_cm = 5000; //max distance is 3000cm = 30meters = 30000mm
    int closest_meas = 5000;
    //for (int scanIndex = 0; scanIndex<542; scanIndex++) {
    for (int scanIndex = 0; scanIndex<541; scanIndex++) {
        unsigned int meas = scanData[scanIndex];
        
        int baseVert = scanIndex*2; //takes two vertices to make a line
        int tipVert = baseVert+1;  //even verts will be (0,0,0), odd will be data
        //double degrees = 315.0 - (double)scanIndex/2.0;  //first 45 are blank, then half degrees
        double degrees = (double)scanIndex/2.0 - 45.0;  //first 45 are blank, then half degrees
        if (degrees < 0) {
            degrees += 360.0;
        }

        double rads = deg2rad(degrees);  //convert to radians for trig 

        int x = meas * cos(rads);
        int y = meas * sin(rads);
        int z = 0;
        
        //width on each side (total width id 166cm = 66 inches=  15+ 36 + 15 
        // 33 inches in cm
        #define x_width 83 

        //distance of the stop field
        // 48 inches in cm
        #define y_forward 122 

        //top speed is 1.1m/s
        #define velocity_cm 110

        //deceleration rate 
        #define decel_cm 25 //20
        
        //this is the distance it takes to decel from max velocity to zero at given rate
        #define y_fullspeed (y_forward + 0.5*(velocity_cm^2)/decel_cm)  
        
        //this is the braking buffer between the decel and stop zones
        // half of the stop size for starters
        #define y_brakezone 150 //61

        // this is the total distance of the stop field plus the decel zone 
        #define y_distance (y_forward+y_brakezone+2*y_fullspeed)

	//saving the angle for the closest measurement
	if((int)meas < closest_meas && y>0){
		closest_meas = meas;
		angle = degrees;
	}

        //stop field
        if (x < x_width && x > -x_width && y < y_forward && y > 0) {
            (*stopVertices)[baseVert].set(0,0,0);  //baseVert will always be (0,0,0) for origin
            (*stopVertices)[tipVert].set(x, y, z);  //data point
            if (y < closest_y_cm) {
                closest_y_cm = y;
            }
        } else {
            (*stopVertices)[baseVert].set(0,0,0);  //baseVert will always be (0,0,0) for origin
            (*stopVertices)[tipVert].set(0,0,0);  //data point        
            stop--;
        }

        //braking zone
        if (x < x_width && x > -x_width && y >= y_forward && y < (y_forward + y_brakezone)) {
            (*brakeVertices)[baseVert].set(0,0,0);  //baseVert will always be (0,0,0) for origin
            (*brakeVertices)[tipVert].set(x, y, z);  //data point
            if (y < closest_y_cm) {
                closest_y_cm = y;
            }
        } else {
            (*brakeVertices)[baseVert].set(0,0,0);  //baseVert will always be (0,0,0) for origin
            (*brakeVertices)[tipVert].set(0,0,0);  //data point        
        }

        //slow field
        if (x < x_width && x > -x_width && y >= (y_forward+y_brakezone) && y < y_distance) {
            (*slowVertices)[baseVert].set(0,0,0);  //baseVert will always be (0,0,0) for origin
            (*slowVertices)[tipVert].set(x, y, z);  //data point
            if (y < closest_y_cm) {
                closest_y_cm = y;
		printf ("closest_y_cm: %d \n",closest_y_cm); 
            }
        } else {
            (*slowVertices)[baseVert].set(0,0,0);  //baseVert will always be (0,0,0) for origin
            (*slowVertices)[tipVert].set(0,0,0);  //data point        
        }
        
        (*outlineVertices)[baseVert].set(px, py, pz);  //data point
        (*outlineVertices)[tipVert].set(x, y, z);  //data point
        px = x;  py = y; pz = z;
        //printf ("scan: %s, index: %d, r: %u, deg: %.1f, rad: %0.4f x: %d, y: %d, z: %d\n", scanNumber, scanIndex, meas, degrees, rads, x, y, z);
    }

    tdist = closest_y_cm - (y_forward+y_brakezone);
    if (tdist < 0){
	tdist = 0;}
 //   speed = sqrt (2.0*tdist * decel_cm);
  //  speed = 7*pow (tdist/(y_distance),2)/3;
    speed = pow(tdist/(2*y_fullspeed),1.5);
    if (speed > 1) speed = 1;
    if (speed < 0) speed = 0;
    if (stop) speed = 0;
    if (speed != speed){
       speed = 0;}

    string response;
    string sendCommand,sendGalilFullCommand;
    string secondCommand;
    //keep sending messages in this loop           
    sendCommand = "p408=";
    // secondCommand = ""
    // sendMessage  = speed;
    char speedChar[21];

    float percent_speed = speed;
    sprintf(speedChar,"%f",percent_speed);
    sendGalilFullCommand = sendCommand+speedChar+"\r"; 
    //printf("The sent command is %s\n",sendGalilFullCommand);
    cout << "Sent command is " << sendGalilFullCommand << "\n" ; 
    //response = getResponseForMsg("QS\r");  //, sizeof locMsg );
    response = getResponseForMsg(sendGalilFullCommand);  //, sizeof locMsg );
    //cout << "Full Response: " <<response << " and count is " << count << endl;
    printf("Full Response: %s \n", response.c_str()); 
    
    printf("epoch: %d, scanNumber: %s, totalBytesRead: %d, stop: %d, closest_y: %d, closest_reading:%d angle: %f speed: %f percent_speed: %f \n", (int)time(0), scanNumber, totalBytesRead, stop, closest_y_cm, closest_meas, angle, speed, percent_speed);


    char output[256] = "";
    sprintf(output,"speed: %f, percent_speed: %f\n", speed, percent_speed);

    myText->setText(output);
    //viewer.setSceneData( root.get() );

 
    //modbus_write_register(ctx, SAFE_SPEED_REGISTER, percent_speed);
    //modbus_write_register(ctx, OBSTRUCTION_ANGLE_REGISTER, ((int)(180 - (int)angle)/10) * 10) ;
    //modbus_write_bit(ctx, BRAKE_COIL, percent_speed);

   /* 
    (*originVertices)[0].set(0,0,0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[1].set(4000, 0, 0);  //data point
    (*originVertices)[2].set(0,0,0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[3].set(0, 8000, 0);  //data point
    (*originVertices)[4].set(0,0,0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[5].set(0, 0, 2000);  //data point
*/
    //lc footprint - must stop outside of this
    (*originVertices)[6].set(-x_width, 0,0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[7].set(x_width,  0, 0);  //data point
    (*originVertices)[8].set(-x_width, 0, 0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[9].set(-x_width, y_forward, 0);  //data point
    (*originVertices)[10].set(x_width, 0,0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[11].set(x_width, y_forward, 0);  //data point
    (*originVertices)[12].set(-x_width, y_forward, 0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[13].set(x_width, y_forward, 0);  //data point
    
    (*originVertices)[14].set(-x_width, 0,0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[15].set(x_width,  0, 0);  //data point
    (*originVertices)[16].set(-x_width, 0, 0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[17].set(-x_width, y_distance, 0);  //data point
    (*originVertices)[18].set(x_width, 0,0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[19].set(x_width, y_distance, 0);  //data point
    (*originVertices)[20].set(-x_width, y_distance, 0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[21].set(x_width, y_distance, 0);  //data point

    (*originVertices)[22].set(-x_width, y_forward+y_brakezone, 0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[23].set(x_width, y_forward+y_brakezone, 0);  //data point


/*   
    (*originVertices)[6].set(0,0,0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[7].set(0, 0, 2000);  //data point
    (*originVertices)[8].set(0,0,0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[9].set(0, 0, 2000);  //data point
    (*originVertices)[10].set(0,0,0);  //baseVert will always be (0,0,0) for origin
    (*originVertices)[11].set(0, 0, 2000);  //data point
  */  

    /*
    (*vertices)[0].set(0,0,0);
    (*vertices)[1].set(0,0,5000);
    (*vertices)[2].set(0,0,0);
    (*vertices)[3].set(10000,10000,10000);
    (*vertices)[4].set(0,0,0);
    (*vertices)[5].set(-10000,-10000,-10000);
    (*vertices)[6].set(0,0,0);
    (*vertices)[7].set(-10000,300,-10000);
    */

}

void updateVerts_old()
{
    int reading = 0;
    for (int degree = 1262; degree > 180; degree-=2) {
        //cout << "Degree: " << degree << " Radian: " << deg2rad(0.5*degree) << " sin: " << 1.0*sin(deg2rad(0.5 * degree)) << " cos: " << 1.0*cos(deg2rad(0.5 * degree)) << endl;
        //(*vertices)[degree].set(0,0,0);
        //(*vertices)[degree+1].set(1.0*sin(deg2rad(0.5 * degree)), 0, 1.0*cos(deg2rad(0.5 * degree)));

        unsigned int meas = scanData[reading] * 10;

        //draw the connected readings outline like the sopas tool
        if (DRAW_OUTLINE) {
            (*stopVertices)[degree].set(meas*sin(deg2rad(0.25 * (degree+720))), 0, meas*cos(deg2rad(0.25 * (degree+720))));
            (*stopVertices)[degree-1].set(meas*sin(deg2rad(0.25 * (degree+720))), 0, meas*cos(deg2rad(0.25 * (degree+720))));
        }

        //draw the current ray from the origin
        if (DRAW_RAYS) {
            (*stopVertices)[degree].set(0,0,0);
            //(*vertices)[degree].set(d.data16bit[i].range[reading]*sin(deg2rad(0.25 * (degree+720))), 0, d.data16bit[i].range[reading]*cos(deg2rad(0.25 * (degree+720))));
            //(*vertices)[degree-1].set(meas*sin(deg2rad(0.25 * (degree+720))), 0, meas*cos(deg2rad(0.25 * (degree+720))));
            (*stopVertices)[degree-1].set(reading, 0, meas);
        }
        
        reading++;
    }
}


class redrawCallback : public osg::NodeCallback
{
public:
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        updateData();
        updateVerts();
        traverse(node, nv);
        //usleep(1);
    }
};


int initViewer()
{
    //Creating the root node
    //osg::ref_ptr<osg::Group> root (new osg::Group);

    //The geode containing our shpae
    //osg::ref_ptr<osg::Geode> myshapegeode (new osg::Geode);

    //Our shape: a capsule, it could have been any other geometry (a box, plane, cylinder etc.)
    //osg::ref_ptr<osg::Capsule> myCapsule (new osg::Capsule(osg::Vec3f(),1,2));

    //Our shape drawable
    //osg::ref_ptr<osg::ShapeDrawable> capsuledrawable (new osg::ShapeDrawable(myCapsule.get()));

    //myshapegeode->addDrawable(capsuledrawable.get());

    // create Geometry object to store all the vertices and lines primitive.
    osg::Geometry* stopLinesGeom = new osg::Geometry();
    osg::Geometry* brakeLinesGeom = new osg::Geometry();
    osg::Geometry* slowLinesGeom = new osg::Geometry();
    osg::Geometry* outlineGeom = new osg::Geometry();
    osg::Geometry* originGeom = new osg::Geometry();
    stopLinesGeom->setUseDisplayList( false );
    brakeLinesGeom->setUseDisplayList( false );
    slowLinesGeom->setUseDisplayList( false );
    outlineGeom->setUseDisplayList( false );
    originGeom->setUseDisplayList( false );

    // pass the created vertex array to the points geometry object.
    stopLinesGeom->setVertexArray(stopVertices);
    brakeLinesGeom->setVertexArray(brakeVertices);
    slowLinesGeom->setVertexArray(slowVertices);
    outlineGeom->setVertexArray(outlineVertices);
    originGeom->setVertexArray(originVertices);
    
    // set the colors as before, plus using the above
    osg::Vec4Array* stopColors = new osg::Vec4Array;
    osg::Vec4Array* brakeColors = new osg::Vec4Array;
    osg::Vec4Array* slowColors = new osg::Vec4Array;
    osg::Vec4Array* outlineColors = new osg::Vec4Array;
    osg::Vec4Array* originColors = new osg::Vec4Array;
    
    stopLinesGeom->setColorArray(stopColors);
    brakeLinesGeom->setColorArray(brakeColors);
    slowLinesGeom->setColorArray(slowColors);
    outlineGeom->setColorArray(outlineColors);
    originGeom->setColorArray(originColors);

    stopLinesGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
    brakeLinesGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
    slowLinesGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
    outlineGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
    originGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
    
    stopColors->push_back(osg::Vec4(1.0f,0.0f,0.0f,1.0f)); //red
    brakeColors->push_back(osg::Vec4(1.0f,1.0f,0.0f,1.0f)); //yellow
    slowColors->push_back(osg::Vec4(0.0f,1.0f,0.0f,1.0f)); //green
    //outlineColors->push_back(osg::Vec4(1.0f,0.0f,0.0f,1.0f)); //red
    outlineColors->push_back(osg::Vec4(0.0f,1.0f,0.0f,1.0f)); //green
    originColors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f)); //white

    // set the normal in the same way color.
    osg::Vec3Array* normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f,0.0f,1.0f));

    stopLinesGeom->setNormalArray(normals);
    stopLinesGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);
    brakeLinesGeom->setNormalArray(normals);
    brakeLinesGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);
    slowLinesGeom->setNormalArray(normals);
    slowLinesGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);
    
    outlineGeom->setNormalArray(normals);
    outlineGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);
    
    originGeom->setNormalArray(normals);
    originGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);

    // This time we simply use primitive, and hardwire the number of coords to use
    // since we know up front,
    if (DRAW_OUTLINE) {
        outlineGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,NUM_VERTS));
        myshapegeode->addDrawable(outlineGeom);
    }

    if (DRAW_RAYS) {
        stopLinesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,NUM_VERTS));
        brakeLinesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,NUM_VERTS));
        slowLinesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,NUM_VERTS));
        myshapegeode->addDrawable(stopLinesGeom);
        myshapegeode->addDrawable(brakeLinesGeom);
        myshapegeode->addDrawable(slowLinesGeom);
    }

    if (DRAW_ORIGIN) {
        originGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,NUM_VERTS));
        myshapegeode->addDrawable(originGeom);
    }

    // add the points geometry to the geode.
    myshapegeode->setDataVariance(osg::Object::DYNAMIC);

    root->addChild(myshapegeode.get());

    //The geode containing our shpae
    osg::ref_ptr<osg::Geode> myTextGeode (new osg::Geode);
    
    //osgText::Text* myText = new osgText::Text();

    // Geode - Since osgText::Text is a derived class from drawable, we 
    // must add it to an osg::Geode before we can add it to our ScenGraph.
    myTextGeode->addDrawable(myText);

    //Set the screen alignment - always face the screen
    myText->setAxisAlignment(osgText::Text::SCREEN);

    //Set the text to our default text string
    myText->setText("Default Text");


    //myText->setPosition(osg::Vec3d(25, 75, 0));
    myText->setPosition(osg::Vec3d(0, 0, 0));
    myText->setColor(osg::Vec4d(1.0f, 1.0f, 1.0f, 1.0f));
    myText->setCharacterSize(48);
    //myText->setFont("./fonts/Vera.ttf");
   
    char output[256] = "";
    sprintf(output, "epoch: %d, scanNumber: %s, totalBytesRead: %d, stop: %d, closest_y: %d, speed: %f\n", (int)time(0), scanNumber, totalBytesRead, stop, closest_y_cm, speed);

    myText->setText(output);

    root->addChild(myTextGeode.get());


    root->setUpdateCallback(new redrawCallback);

    viewer.setSceneData( root.get() );
    //viewer.setThreadingModel(osgViewer::Viewer::ThreadingModel::SingleThreaded);

    //Stats Event Handler s key
    //viewer.addEventHandler(new osgViewer::StatsHandler);

    //Windows size handler
    //viewer.addEventHandler(new osgViewer::WindowSizeHandler);

    // add the state manipulator
    //viewer.addEventHandler( new       osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );

    //The viewer.run() method starts the threads and the traversals.
    return (viewer.run());
}



int main ()
{
/*    
//MODBUS INIT
    ctx = modbus_new_tcp(IP_ADDRESS, PORT);

    if (modbus_connect(ctx) == -1) {
        //fprintf(stderr, "Connexion failed: %s\n",modbus_strerror(errno));
        printf("\n\n MODBUS CONNECTION FAILED");
        modbus_free(ctx);
        return -1;
    }
    else
    printf("\n MODBUS CONNECTED!!");
*/

    memset(&host_info, 0, sizeof host_info);
    host_info.ai_family   = AF_UNSPEC;     
    host_info.ai_socktype = SOCK_STREAM; 
    status = getaddrinfo("10.0.0.7", "1001", &host_info, &host_info_list);
    // getaddrinfo returns 0 on succes, or some other value when an error occured.
    if (status != 0)  printf ("getaddrinfo ", gai_strerror(status));
    printf ("Creating a socket...\n");
    socketfd = socket(host_info_list->ai_family, host_info_list->ai_socktype,
                      host_info_list->ai_protocol);
    if (socketfd == -1)  printf ("socket error ");
    printf ("Connecting to the sensor's socket...\n");
    status = connect(socketfd, host_info_list->ai_addr, host_info_list->ai_addrlen);
    if (status == -1)  printf ("connect error\n") ;
    else printf ("Connection successful, lets go grab some data\n") ;
    
    /*cout << "Receiving complete. Closing socket..." << endl;
    freeaddrinfo(host_info_list);
    close(socketfd);
    return 0;*/

#define SERIAL_ONLY_TEST 0
#if SERIAL_TEST
    while (1) {    
        updateData();
        for (int range=1; range<542; range++) {
            printf("%d %u ", range, scanData[range]);

        }
        printf("\n");
    }
    return 0;
#endif 

    initSerial();
    updateData();
    updateVerts();
    initViewer();
while(1){
    updateData();
    updateVerts();
}
    return 0;
}
