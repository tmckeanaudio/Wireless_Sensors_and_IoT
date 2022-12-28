// MobileSink.cc
// Author: Tyler McKean
// Created on: Nov 27, 2021
// C++ File that initializes and implements the Mobile Sink module needed for the
// WSN/IoT Homework 2 requirements which simulates a Dual-Beacon Discovery
// functionality as part of the Stop-and-Wait ARQ protocol

#include <stdio.h>
#include <string.h>
#include <omnetpp.h>
#include <math.h>

using namespace omnetpp;
// Define Mobile Sink Node module and all of its parameters and events
class MobileSinkNode2BD : public cSimpleModule
{
  private:
    // Declare Parameters and Variables
    double T_bi;
    double R; // Discovery Range
    double r; // Communication Range
    double speed; // Speed is 40Km/hr or 11.11m/s
    double delta; // Delta is 1ms
    double theta; // angle between Starting and Ending Coordinates
    double x_s, x_e, x_c; // start, end, and current X Coordinates
    double y_s, y_e, y_c; // start, end, and current Y Coordinates
    int correctRx;
    int lastDistinctNoRx;
    int distinctPacketsSentCurrentPassage;
    // Declare Events
    cMessage *SRBtoSend;
    cMessage *LRBtoSend;
    cMessage *MoveMS;
  public:
    MobileSinkNode2BD();
    virtual ~MobileSinkNode2BD();
  protected:
    // The following redefined virtual function holds the algorithm.
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void updatePosition();
    virtual void sendBeacon(char beaconType);
    virtual void sendAck();
    virtual double computeTheta();
};
// The module class needs to be registered with OMNeT++
Define_Module(MobileSinkNode2BD);
// Mobile Sink Constructor
MobileSinkNode2BD::MobileSinkNode2BD(){
    SRBtoSend = nullptr;
    LRBtoSend = nullptr;
    MoveMS = nullptr;
}
// Mobile Sink Destructor
MobileSinkNode2BD::~MobileSinkNode2BD(){
    cancelAndDelete(SRBtoSend);
    cancelAndDelete(LRBtoSend);
    cancelAndDelete(MoveMS);
}

void MobileSinkNode2BD::initialize(){
    cModule *c = getModuleByPath("dualBeacon");
    R = ((double)c->par("R"));
    x_s = -(R + 1); y_s = 15; // S = (-101 or -201,15) should start 1m outside DR
    x_e = (R + 1); y_e = 15; // E = (101 or 201,15) should end 1m outside DR
    x_c = x_s; // Set current X coordinate to starting position
    y_c = 15;
    speed = par("speed"); // 11.111 m/s
    delta = par("delta"); // 1ms
    T_bi = 0.1;

    // Print Out Starting X,Y position of Mobile Sink
    EV << "MS starting at ("<<x_s<<","<<y_s<<")"<< endl;
    EV << "MS ending at ("<<x_e<<","<<y_e<<")"<< endl;
    correctRx = 0; // # of correct received data packets
    distinctPacketsSentCurrentPassage = 0;
    lastDistinctNoRx = -1; //

    theta = computeTheta(); // angle between starting position (xs,ys) and ending position (xe,ye)
    // start new passage
    if ((int)c->par("numPassages") < ((int)c->par("totalPassages"))) // Number of passages still lower than targeted amount of passages for simulation
    {
        // schedule sink movement
        EV << "Move Sink Position" << endl;
        if(MoveMS != nullptr){
            cancelAndDelete(MoveMS);}
        MoveMS = new cMessage("MoveMS"); // Create new Move Sink Event
        scheduleAt(simTime() + delta, MoveMS); // Schedule event at next Delta instance
        // schedule LRB
        EV << "Schedule LRB" << endl;
        LRBtoSend = new cMessage("LRBtoSend");
        scheduleAt(simTime(), LRBtoSend);
        // schedule SRB
        EV << "Schedule SRB" << endl;
        SRBtoSend = new cMessage("SRBtoSend");
        scheduleAt(simTime() + T_bi, SRBtoSend);
    }
}
void MobileSinkNode2BD::updatePosition() // Update Mobile Sink position function
{
    double new_x; // new X coordinate variable
    double new_y; // new Y coordinate variable
    bool discPhase;
    bool commPhase;

    cModule *c = getModuleByPath("dualBeacon");
    double x_c = ((double)c->par("x_ms"));
    double y_c = ((double)c->par("y_ms"));

    double displ = speed * delta; // Displacement will be product of speed (11.11 m/s) with Delta (1ms)
    double x_displ = displ * cos(theta * PI / 180.0); // X coordinate displacement calculation
    double y_displ = displ * sin(theta * PI / 180.0); // Y coordinate displacement calculation

    if (x_s <= x_e) // Check if X Starting Position is less than X Ending Position
    {
        EV << "Mobile Sink moved "<< displ << " meters along X-axis"<< endl;
        new_x = x_c + x_displ; // Add X displacement with X current

        if (new_x >= x_e) // Check if new X large than X Ending Position
        {
            new_x = x_e; // Set X New equal to X End
        }
    }
    else
    {
        new_x = x_c - x_displ; // Allows any X direction not just Left to Right Movement, in this case check is Right to Left movement

        if (new_x <= x_e)
        {
            new_x = x_e;
        }
    }

     if (y_s <= y_e) // Check if Y Starting Position is less than Y Ending Position
    {
        new_y = y_c + y_displ; // Add Y displacement with X current

        if (new_y >= y_e) // Check if new Y large than Y Ending Position
        {
            new_y = y_e; // Set Y New equal to Y End
        }
    }
    else
    {
        new_y = y_c - y_displ; // Allows any Y direction not just Left to Right Movement, in this case check is Right to Left movement

        if (new_y <= y_e)
        {
            new_y = y_e;
        }
    }
    // Update Discovery Phase variable if MS has entered region
    if(new_x >= -(R) and new_x <= R){
        EV << "Mobile Sink is in Discovery Range"<< endl;
        discPhase = true;
        (c->par("in_discovery_phase")) = discPhase;
    }
    // Update Communication Phase variable if MS has entered region
    if(new_x >= -(r) and new_x <= r){
        EV << "Mobile Sink is in Communication Range"<< endl;
        commPhase = true;
        (c->par("in_communication_phase")) = commPhase;
    }
    // sink arrived at destination
    if (new_x == x_e && new_y == y_e)
    {
        // reset position
        x_c = x_s;
        y_c = y_s;
        // increase passages counter
        c->par("numPassages") = ((int)c->par("numPassages") + 1);
        // reset counters regarding current passage

        // reset in_discovery_phase variable
        c->par("in_discovery_phase") = false;
    }

    EV << "Mobile Sink Location is now at ("<< new_x<<","<< new_y <<")" << endl;
    c->par("x_ms") = new_x; // Save to Network x-coordinate
    c->par("y_ms") = new_y; // Save to Network y-coordinate
}
void MobileSinkNode2BD::sendBeacon(char beaconType) // Send beacon function
{
    // short range beacon
    if (beaconType == 'S')
    {
        // send srb
        EV << "Mobile Sink Sending SRB" << endl;
        cMessage *SRB = new cMessage("SRB"); // generate new cMessage for the SRB
        send(SRB, "out"); // send out to Wireless Channel
    }
    // long range beacon
    else if (beaconType == 'L')
    {
        // send lrb
        EV << "Mobile Sink Sending LRB" << endl;
        cMessage *LRB = new cMessage("LRB"); // generate new cMessage for LRB
        send(LRB, "out"); // send out to Wireless Channel
    }
}
void MobileSinkNode2BD::sendAck() // Function for sending acknowledgments to Wireless Channel
{
    cMessage *ACK = new cMessage("ACK"); // generate new cMessage for acknowledgment
    send(ACK, "out"); // send out to Wireless Channel
}
double MobileSinkNode2BD::computeTheta()
{
    double ip = sqrt(pow(x_e - x_s, 2) + pow(y_e - y_s, 2));
    double cat = std::abs(x_e - x_s);
    double theta_val = acos(cat / ip) * 180.0 / PI;
    return theta_val;
}
void MobileSinkNode2BD::handleMessage(cMessage *msg){
    cModule *c = getModuleByPath("dualBeacon");
    if (msg == SRBtoSend and ((int)c->par("numPassages") < (int)c->par("totalPassages"))) // Self-message to send SRB
    {
        // send beacon
        sendBeacon('S'); // Function to transmit SRB
        scheduleAt(simTime() + 0.0000001 + 2.0 * T_bi, SRBtoSend); // Scheduling SRB event
    }
    else if (msg == LRBtoSend and ((int)c->par("numPassages") < (int)c->par("totalPassages"))) // Self-message to send LRB
    {
        // send beacon
        sendBeacon('L'); // Function to transmit LRB
        scheduleAt(simTime() + 0.000001 + 2.0 * T_bi, LRBtoSend); // Scheduling LRB event
    }
    else if (msg == MoveMS and ((int)c->par("numPassages") < (int)c->par("totalPassages"))) // Self-message to move Mobile Sink
    {
        updatePosition(); // Function to update Mobile Sink position
        if(MoveMS != nullptr){
            cancelAndDelete(MoveMS);
        }
        MoveMS = new cMessage("MoveMS"); // Create new Move Sink Event
        scheduleAt(simTime() + delta, MoveMS); // Schedule event at next Delta instance
    }
    else if ( ((std::string) msg->getName()) == "dataPacket") // If received event is a data packet
    {
        // increase counter
        if (lastDistinctNoRx < distinctPacketsSentCurrentPassage) // Check for corrupted data packets
        {
            lastDistinctNoRx = distinctPacketsSentCurrentPassage; //
            correctRx++; // increase # of received packets
        }
        // send ACK
        EV << "Mobile Sink Sending ACK" << endl;
        sendAck(); // received packet, send acknowledgment back to Sensor Node
        delete msg;
    }
}
