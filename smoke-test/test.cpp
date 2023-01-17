/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini <ugo.pattacini@iit.it>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
*/

#include <string>

#include <robottestingframework/dll/Plugin.h>
#include <robottestingframework/TestAssert.h>

#include <yarp/robottestingframework/TestCase.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

using namespace std;
using namespace robottestingframework;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

/**********************************************************************/
class TestAssignmentGraspIt : public yarp::robottestingframework::TestCase,
                              public PortReader
{
    RpcClient portBall;
    RpcClient portGI;
    Port      portHandR;
    Port      portHandL;

    Vector ballPosRobFrame;
    bool hit;

    /******************************************************************/
    Vector getBallPosition()
    {
        Bottle cmd,reply;
        cmd.addVocab32("get");
        if (!portBall.write(cmd,reply))
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to talk to world");
        if ((reply.get(0).asVocab32()!=Vocab32::encode("ack")) || (reply.size()<4))
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Invalid reply from world");

        Vector pos(3);
        pos[0]=reply.get(1).asFloat64();
        pos[1]=reply.get(2).asFloat64();
        pos[2]=reply.get(3).asFloat64();

        return pos;
    }

    /******************************************************************/
    bool setBallPosition(const Vector& pos)
    {
        if (pos.length()>=3)
        {
            Bottle cmd,reply;
            cmd.addVocab32("set");
            cmd.addFloat64(pos[0]);
            cmd.addFloat64(pos[1]);
            cmd.addFloat64(pos[2]);
            if (!portBall.write(cmd,reply))
                ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to talk to world");
            if (reply.get(0).asVocab32()!=Vocab32::encode("ack"))
                ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Invalid reply from world");
            return true;
        }
        else
            return false;
    }

public:
    /******************************************************************/
    TestAssignmentGraspIt() :
        yarp::robottestingframework::TestCase("TestAssignmentGraspIt"),
        hit(false)
    {
    }

    /******************************************************************/
    virtual ~TestAssignmentGraspIt()
    {
    }

    /******************************************************************/
    virtual bool setup(yarp::os::Property& property)
    {
        string robot=property.check("robot",Value("icubSim")).asString();
        float rpcTmo=(float)property.check("rpc-timeout",Value(240.0)).asFloat64();

        string robotPortRName("/"+robot+"/cartesianController/right_arm/state:o");
        string robotPortLName("/"+robot+"/cartesianController/left_arm/state:o");

        string portBallName("/"+getName()+"/ball:rpc");
        string portGIName("/"+getName()+"/gi:rpc");
        string portHandRName("/"+getName()+"/hand/right:i");
        string portHandLName("/"+getName()+"/hand/left:i");

        portBall.open(portBallName);
        portGI.open(portGIName);
        portHandR.open(portHandRName);
        portHandL.open(portHandLName);

        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Set rpc timeout = %g [s]",rpcTmo));
        portBall.asPort().setTimeout(rpcTmo);
        portGI.asPort().setTimeout(rpcTmo);

        Time::delay(5.0);

        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Connecting Ports");

        if (!Network::connect(portBallName,"/assignment_grasp-it-ball/rpc"))
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to connect to /assignment_grasp-it-ball/rpc");

        if (!Network::connect(portGIName,"/service"))
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to connect to /service");

        if (!Network::connect(robotPortRName,portHandRName))
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL(Asserter::format("Unable to connect to %s",robotPortRName.c_str()));

        if (!Network::connect(robotPortLName,portHandLName))
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL(Asserter::format("Unable to connect to %s",robotPortLName.c_str()));

        Rand::init();

        return true;
    }

    /******************************************************************/
    virtual void tearDown()
    {
        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Closing Ports");
        portBall.close();
        portGI.close();
        portHandL.close();
        portHandR.close();
    }

    /******************************************************************/
    virtual bool read(ConnectionReader& reader)
    {
        if (!hit)
        {
            Bottle data;
            data.read(reader);

            Vector x(3);
            x[0]=data.get(0).asFloat64();
            x[1]=data.get(1).asFloat64();
            x[2]=data.get(2).asFloat64();

            double d=norm(ballPosRobFrame-x);
            if (d<0.1)
            {
                ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Great! We're at %g [m] from the ball",d));
                hit=true;
            }
        }

        return true;
    }

    /******************************************************************/
    virtual void run()
    {
        Time::delay(5.0);

        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Retrieving initial ball position");
        Vector initialBallPos=getBallPosition();
        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("initial ball position = (%s) [m]",
                                         initialBallPos.toString(3,3).c_str()));

        Vector min(3,0.0),max(3,0.0);
        min[0]=-0.02; max[0]=0.0;   // x-axis
        min[1]=-0.05; max[1]=0.05;  // y-axis
        min[2]=0.0;   max[2]=0.0;   // z-axis

        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Setting new initial ball position");
        initialBallPos+=Rand::vector(min,max);
        setBallPosition(initialBallPos);
        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("new ball position = (%s) [m]",
                                         initialBallPos.toString(3,3).c_str()));

        // compute ball position in robot's root frame
        ballPosRobFrame=initialBallPos;
        ballPosRobFrame[2]-=0.63;

        Bottle cmd,reply;
        cmd.addString("look_down");
        if (!portGI.write(cmd,reply))
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to talk to GI");
        if (reply.get(0).asString()!="ack")
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to look_down");
        cmd.clear(); reply.clear();

        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Proximity check is now active");
        portHandR.setReader(*this);
        portHandL.setReader(*this);

        cmd.addString("grasp_it");
        if (!portGI.write(cmd,reply))
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to talk to GI");
        if (reply.get(0).asString()!="ack")
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to grasp_it");
        cmd.clear(); reply.clear();

        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Retrieving final ball position");
        Vector finalBallPos=getBallPosition();
        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("final ball position = (%s) [m]",
                                          finalBallPos.toString(3,3).c_str()));

        double d=finalBallPos[2]-initialBallPos[2];
        ROBOTTESTINGFRAMEWORK_TEST_CHECK(hit,"We've approached the ball!");
        ROBOTTESTINGFRAMEWORK_TEST_CHECK(d>=0.02,Asserter::format("Ball has been lifted for at least %g [m]!",d));
    }
};

ROBOTTESTINGFRAMEWORK_PREPARE_PLUGIN(TestAssignmentGraspIt)
