// CSMA_CA.cc
// Author: Tyler McKean
// Created on: Oct 20, 2021
// C++ File that initializes and creates all the modules needed for the
// WSN/IoT Homework 1 requirements
//

#include <stdio.h>
#include <string.h>
#include <omnetpp.h>
#include <math.h>

using namespace omnetpp;
// Define Sensors Node module and all of its parameters and events
class SensorNodeCSMACA : public cSimpleModule
{
  private:
    // Declare Parameters and Variables
    volatile int NB;
    volatile int BE;
    int macMinBE;
    int macMaxBE;
    int macMaxCSMABackoffs;
    double Dp;
    double D_bp;
    double T_CCA;
    double energy;
    double latency;
    double Ptx;
    double Prx;
    double packetCreationTime;
    int packets2send;
    int totalPackets;
    // Declare Events
    cMessage *backoffExpired;
    cMessage *setChannelBusy;
    cMessage *setChannelFree;
    cMessage *sendMessage;
    cMessage *decreaseTxCounter;
  public:
    SensorNodeCSMACA();
    virtual ~SensorNodeCSMACA();
  protected:
    // The following redefined virtual function holds the algorithm.
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void decrease_and_repeat();
    virtual bool performCCA();
    virtual void setChannelState(bool state);
    virtual double create_backoff_time();
    //virtual void finish() override;
};
Define_Module(SensorNodeCSMACA);
// Define Sink Node module and its parameters
class SinkNodeCSMACA : public cSimpleModule
{
  private:
    int RxPackets;
    int numCollided;
  protected:
    // The following redefined virtual function holds the algorithm.
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
};
// The module class needs to be registered with OMNeT++
Define_Module(SinkNodeCSMACA);

// Sensor Node Constructor
SensorNodeCSMACA::SensorNodeCSMACA(){
    backoffExpired = nullptr;
    setChannelBusy = nullptr;
    setChannelFree = nullptr;
    sendMessage = nullptr;
    decreaseTxCounter = nullptr;
}
// Sensor Node Destructor
SensorNodeCSMACA::~SensorNodeCSMACA(){
    cancelAndDelete(backoffExpired);
    cancelAndDelete(setChannelBusy);
    cancelAndDelete(setChannelFree);
    cancelAndDelete(sendMessage);
    cancelAndDelete(decreaseTxCounter);
}

void SensorNodeCSMACA::initialize() {
    NB = 0;
    macMinBE = par("macMinBE"); BE = macMinBE;
    macMaxCSMABackoffs = par("macMaxCSMABackoffs");
    packets2send = par("packets2send");
    totalPackets = par("totalPackets");
    packetCreationTime = par("packetCreationTime");
    D_bp = 0.00032;
    Dp = 0.004256;
    T_CCA = (D_bp/20)*8;
    Prx = 56.4;
    Ptx = 49.5;
    energy = 0;
    latency = 0;
    // Create packet creation time
    if(packets2send > 0){
        packetCreationTime = simTime().dbl();
    }
    // Clear Backoff Timer since channel should be free at start
    if(backoffExpired != nullptr){
        cancelAndDelete(backoffExpired);}
    backoffExpired = new cMessage("backoffExpired");
    scheduleAt(simTime() + create_backoff_time(), backoffExpired);
}

void SensorNodeCSMACA::handleMessage(cMessage *msg){
    if(msg == backoffExpired){
        // Backoff Timer expired, Perform CCA and Set Channel Busy
        EV << "Backoff Timer Expired" << endl;
        if(performCCA()){
            if(setChannelBusy != nullptr){
                cancelAndDelete(setChannelBusy);} // Avoid Memory Leak
            setChannelBusy = new cMessage("setChannelBusy"); //setChannelBusy
            scheduleAt(simTime() + D_bp - 0.000001, setChannelBusy);
        }
        else{
            // Channel BUSY, Increase Backoff Exponential and Number of Backoffs
            BE++;
            NB++;
            if(BE > macMaxBE){
                // Make sure Backoff Exponential doesn't exceed MacMaxBE value
                BE = macMaxBE;
            }
            if(NB <= macMaxCSMABackoffs){
                // Schedule another Backoff Timer
                scheduleAt(simTime() + D_bp + create_backoff_time(), backoffExpired);
            }
            else{
                // Increase Dropped Packet Parameter and repeat process
                cModule *c = getModuleByPath("CSMA_CA");
                c->par("numDroppedPackets") = ((int)c->par("numDroppedPackets") + 1);
                decrease_and_repeat();
            }
        }
    }
    else if(msg == setChannelBusy){
        // Change Channel from FREE to BUSY
        EV << "Setting Channel Busy" << endl;
        setChannelState(false);
        if(sendMessage != nullptr){
            cancelAndDelete(sendMessage);} // Avoid Memory Leak
        sendMessage = new cMessage("sendMessage");
        scheduleAt(simTime() + 0.000001, sendMessage);
    }
    else if(msg == setChannelFree){
        // Change Channel from BUSY to FREE
        EV << "Setting Channel Free" << endl;
        setChannelState(true);
        cModule *c = getModuleByPath("CSMA_CA");
        if((int)c->par("concurrentTransmissions") <= 1){
            // Calculate latency after successful packet transmission
            double tmp_latency = 0;
            tmp_latency = simTime().dbl() - packetCreationTime; //record when schedule packet
            c->par("latency") = ((double)c->par("latency") + tmp_latency);
        }
        if(decreaseTxCounter != nullptr){
            cancelAndDelete(decreaseTxCounter);} // Avoid Memory Leak
        decreaseTxCounter = new cMessage ("decreaseTxCounter");
        scheduleAt(simTime() + 0.000001, decreaseTxCounter);
    }
    else if(msg == sendMessage){
        // Sending Message, Calculate Energy, Send Data Packet
        EV << "Sending Message" << endl;
        cModule *c = getModuleByPath("CSMA_CA");
        c->par("energy") = ((double)c->par("energy") + Ptx * (double)Dp);
        c->par("concurrentTransmissions") = ((int)c->par("concurrentTransmissions") + 1);
        c->par("numTxPackets") = ((int)c->par("numTxPackets") + 1);
        cMessage *dataPacket = new cMessage;
        // Packet Creation time at time data packet created
        simtime_t packetCreation_t = simTime() - dataPacket->getCreationTime();
        send(dataPacket,"out");
        if(setChannelFree != nullptr){
            cancelAndDelete(setChannelFree);} // Avoid Memory Leak
        setChannelFree = new cMessage("setChannelFree");
        scheduleAt(simTime() + Dp, setChannelFree);
    }
    else if(msg == decreaseTxCounter){
        // Channel Free, Decrease Concurrent Tx Value
        EV << "Decreasing Concurrent Tx Counter" << endl;
        cModule *c = getModuleByPath("CSMA_CA");
        c->par("concurrentTransmissions") = ((int)c->par("concurrentTransmissions") - 1);
        decrease_and_repeat();
    }
}

void SinkNodeCSMACA::initialize(){
    // Initialize Sink Node parameters
    RxPackets = 0;
    numCollided = 0;
}
void SinkNodeCSMACA::handleMessage(cMessage *msg){
    // Either increase Collided Packet # or Received Packet #
    cModule *c = getModuleByPath("CSMA_CA");
    if((int)c->par("concurrentTransmissions") > 1){
        numCollided++;
    }
    else{
        RxPackets++;
    }
    cancelAndDelete(msg);
}
void SinkNodeCSMACA::finish(){
    // Perform calculations of Network parameters
    cModule *c = getModuleByPath("CSMA_CA");
    int totPackets = RxPackets + numCollided + ((int)c->par("numDroppedPackets"));
    double DR = ((double)RxPackets)/((double)totPackets)*100;
    double LAT = ((double)c->par("latency")/RxPackets)*1000;
    double networkEnergy = ((double)c->par("energy")/RxPackets);

    EV << "Total Number of Packets was: "<< totPackets << endl;
    EV << "The Average Delivery Ratio was: "<< DR << "%" << endl;
    EV << "The Average Packet Latency was: "<< LAT << "msecs" << endl;
    EV << "The Average Energy Consumption was: " << networkEnergy << "mJoules" << endl;
}
void SensorNodeCSMACA::decrease_and_repeat(){
    // Reinitialize parameters, decrease Packet # and Schedule Backoff Timer for 5 secs
    int T = 5;
    NB = 0;
    BE = macMinBE;
    packets2send--;
    if(packets2send > 0){
        if(backoffExpired != nullptr){
            cancelAndDelete(backoffExpired);}
        backoffExpired = new cMessage("backoffExpired");
        double tmp = (totalPackets - packets2send) * T + create_backoff_time();
        packetCreationTime = tmp;
        scheduleAt(packetCreationTime, backoffExpired);
        EV << "Decreasing Packets and Repeating Process" << endl;
    }
}
bool SensorNodeCSMACA::performCCA(){
    // Perform Clear Channel Assessment
    cModule *c = getModuleByPath("CSMA_CA");
    c->par("energy") = ((double)c->par("energy")+Prx*(double)T_CCA);
    return((bool)c->par("channelFree"));
}
void SensorNodeCSMACA::setChannelState(bool state){
    // Change Channel State from BUSY/IDLE
    cModule *c = getModuleByPath("CSMA_CA");
    c->par("channelFree") = state;
}
double SensorNodeCSMACA::create_backoff_time(){
    // Generate random uniform integer based on backoff timer
    int RV = intuniform(0,pow(2,BE));
    double tmp = ((double)RV)*D_bp;
    return tmp;
}

