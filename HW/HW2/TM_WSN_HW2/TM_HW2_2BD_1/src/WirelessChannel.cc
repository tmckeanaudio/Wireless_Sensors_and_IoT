// WirelessChannel.cc
// Author: Tyler McKean
// Created on: Nov 27, 2021
// C++ File that initializes the Wireless Channel module needed for the
// WSN/IoT Homework 2 requirements which simulates a Dual-Beacon Discovery
// functionality as part of the Stop-and-Wait ARQ protocol

#include <stdio.h>
#include <string.h>
#include <omnetpp.h>
#include <math.h>

using namespace omnetpp;
// Define Wireless Channel module and all of its parameters and events
class WirelessChannel : public cSimpleModule
{
  private:
    // Declare Parameters and Variables
    bool commPhase;
    bool discPhase;
    double R; // Discovery Range
    double r; // Communication Range
    double x_c; // X coordinate of SN = 0
    double y_c; // Y coordinate of SN = 0
    // Declare Events
  public:
    //WirelessChannel();
    //virtual ~WirelessChannel();
  protected:
    // The following redefined virtual function holds the algorithm.
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual bool calculateMessageLoss();
};
Define_Module(WirelessChannel);

void WirelessChannel::initialize(){
    cModule *c = getModuleByPath("dualBeacon");
    R = ((double)c->par("R"));
    r = ((double)c->par("r"));
    x_c = ((double)c->par("x_ms"));
    y_c = ((double)c->par("y_ms"));
}
void WirelessChannel::handleMessage(cMessage *msg){
    bool msgCorrupt = calculateMessageLoss();
    EV << "The " << (std::string) msg->getName() <<" was Corrupt = "<< msgCorrupt << endl;
    if((((std::string) msg->getName()) == "LRB") and msgCorrupt == 0 and discPhase == true){
        EV << "Wireless Channel Received LRB From Mobile Sink and Sending to Sensor Node" << endl;
        cMessage *LRB = new cMessage("LRB");
        send(LRB,"out_SN");
    }
    else if((((std::string) msg->getName()) == "SRB") and calculateMessageLoss() == false and commPhase == true){
        EV << "Wireless Channel Received SRB From Mobile Sink and Sending to Sensor Node" << endl;
        cMessage *SRB = new cMessage("SRB");
        send(SRB,"out_SN");
    }
    else if((((std::string) msg->getName()) == "dataPacket") and calculateMessageLoss() == false and commPhase == true){
        EV << "Wireless Channel Received Data Packet from Sensor Node and Sending to Mobile Sink" << endl;
        cMessage *dataPacket = new cMessage("dataPacket");
        send(dataPacket,"out_MS");
    }
    else {
        EV << (std::string) msg->getName() << " Corrupted by Wireless Channel" << endl;
        delete msg;
    }
}
bool WirelessChannel::calculateMessageLoss(){
    double x = 0; // X Coordinate of SN
    double y = 0; // Y Coordinate of SN
    double p = 0.0; // Message Loss Probability
    double d = 0.0; // Euclidean distance
    double tmp;
    bool msgCorrupt;
    cModule *c = getModuleByPath("dualBeacon");
    x_c = ((double)c->par("x_ms"));
    y_c = ((double)c->par("y_ms"));
    if((bool)c->par("in_discovery_phase") == false){
        EV << "Mobile Sink Not In Discovery Range, So Beacon Was Corrupted"<< p << endl;
        p = 1;
        msgCorrupt = true;
    } else { // Calculate the Message Loss Probability by taking the Euclidean distance of MS and SN
        d = sqrt(pow(x_c - x, 2) + pow(y_c - y, 2));
        p = d/(4*R);
        EV << "Message Loss Probability is"<< p << endl;
        tmp = uniform(0,1); // Use RV to determine if msg is corrupted
        EV << "Random Variable is "<< tmp << endl;
        if(tmp > p){
            msgCorrupt = false; // Msg not corrupted
        }
        if (tmp < p){
            msgCorrupt = true; // Msg corrupted
        }
    }
    return msgCorrupt;
}
