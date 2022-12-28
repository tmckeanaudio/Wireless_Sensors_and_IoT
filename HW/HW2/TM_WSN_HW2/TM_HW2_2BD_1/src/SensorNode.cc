// SensorNode.cc
// Author: Tyler McKean
// Created on: Nov 27, 2021
// C++ File that initializes and implements the Sensor Node module needed for the
// WSN/IoT Homework 2 requirements which simulates a Dual-Beacon Discovery
// functionality as part of the Stop-and-Wait ARQ protocol

#include <stdio.h>
#include <string.h>
#include <omnetpp.h>
#include <math.h>

using namespace omnetpp;
// Define Sensor Node module and all of its parameters and events
class SensorNode2BD : public cSimpleModule
{
  private:
    // Declare Parameters and Variables
    bool radioOn;
    bool lowDutyCycle;
    double R; // Discovery Range 100m
    double r; // Communication Range 50m
    double T_bi;
    double T_on;
    double T_off_low;
    double T_off_high;
    double deltaLow;
    double deltaHigh;
    double txTimeout;
    double packetLength;
    double sigma;
    int timesDiscovered;
    int ackLost;
    int ackPackets;
    int distinctPacketsSentCurrentPassage;
    int numPassages;
    int totalPassages;
    double energyDiscovery;
    double energyTransfer;
    double Prx;
    double Ptx;
    double ackDuration;
    double packetDuration;
    double tmpTime;
    // Declare Events
    cMessage *turnRadioOn;
    cMessage *turnRadioOff;
    cMessage *returnToLowDutyCycle;
    cMessage *sendData;
    cMessage *txTimeoutExpired;
  public:
    SensorNode2BD();
    virtual ~SensorNode2BD();
  protected:
    // The following redefined virtual function holds the algorithm.
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void computeTimeouts();
    virtual void setInitialRadioState();
    virtual void changeRadioState(bool state);
    virtual void finish() override;
};
Define_Module(SensorNode2BD);
// Sensor Node Constructor
SensorNode2BD::SensorNode2BD(){
    // Declare events as nullptrs
    turnRadioOn = nullptr;
    turnRadioOff = nullptr;
    returnToLowDutyCycle = nullptr;
    sendData = nullptr;
    txTimeoutExpired = nullptr;
}
// Sensor Node Destructor
SensorNode2BD::~SensorNode2BD(){
    // cancelAndDelete all events initialized in Constructor
    cancelAndDelete(turnRadioOn);
    cancelAndDelete(turnRadioOff);
    cancelAndDelete(returnToLowDutyCycle);
    cancelAndDelete(sendData);
    cancelAndDelete(txTimeoutExpired);
}
// Define Wireless Channel module and all of its parameters and events

void SensorNode2BD::initialize(){
    cModule *c = getModuleByPath("dualBeacon");
    lowDutyCycle = true; // Start SN in low duty cycle
    T_bi = par("T_bi");
    deltaLow = (c->par("deltaLow"));
    deltaHigh = (c->par("deltaHigh"));
    packetDuration = par("packetDuration");
    ackDuration = par("ackDuration");
    timesDiscovered = par("timesDiscovered");
    ackLost = par("ackLost");
    ackPackets = par("ackPackets");
    Prx = par("Prx");
    Ptx = par("Ptx");
    energyDiscovery = par("energyDiscovery");
    energyTransfer = par("energyTransfer");

    T_on = 2.0 * T_bi; // Period radio will be ON
    T_off_low = T_on * (1.0 - deltaLow) / deltaLow; // Period radio off for low duty cycle
    T_off_high = T_on * (1.0 - deltaHigh) / deltaHigh; // Period radio off for high duty cycle
    txTimeout = 2.0 * sigma + ackDuration + packetDuration; // Tx timeout for reception of acknowledgments

    //computeTimeouts(); // Function to compute the timeouts

    // turn radio on/off (start initial duty cycle)
    setInitialRadioState(); // Set random state for Sensor Node radio to be ON/OFF
}
void SensorNode2BD::handleMessage(cMessage *msg){
    if (msg == turnRadioOn && numPassages < totalPassages)
    {
        EV << "Turn Radio On" << endl;
        changeRadioState(true);

        scheduleAt(simTime() + T_on, turnRadioOff);

        // update energy spent to rx
        cModule *c = getModuleByPath("dualBeacon");
        if ((bool)c->par("in_discovery_phase"))
        {
            cModule *c = getModuleByPath("dualBeacon");
            energyDiscovery += Prx * T_on;
            tmpTime = simTime().dbl();
        }
    }
    else if (msg == turnRadioOff && numPassages < totalPassages)
    {
        EV << "Turn Radio Off" << endl;
        changeRadioState(false);
        if (lowDutyCycle)
            scheduleAt(simTime() + T_off_low, turnRadioOn);
        else
            scheduleAt(simTime() + T_off_high, turnRadioOn);
    }
    else if ( ((std::string) msg->getName()) == "SRB" )
    {
        EV << "Sensor Node Received SRB" << endl;
        if (radioOn)
        {
            // update counter of contacts during the current passage


            // update discovers counter
            timesDiscovered++;

            // cancel radio-off of old duty cycle
            if(turnRadioOff != nullptr){
                cancelAndDelete(turnRadioOff);
            }

            // cancel lrb timeout
            if(txTimeoutExpired != nullptr){
                cancelAndDelete(txTimeoutExpired);
            }

            // schedule data transmission
            if(sendData != nullptr){
                cancelAndDelete(sendData);
            }
            sendData = new cMessage("send_data");
            scheduleAt(simTime(), sendData);
            cModule *c = getModuleByPath("dualBeacon");
            if (c->par("in_discovery_phase"))
            {
                // remove extra time from discovery energy
                energyDiscovery = std::max(0.0,energyDiscovery - Prx * (T_on - (simTime().dbl() - tmpTime)));
                // update discovery phase variable
                c->par("in_discovery_phase") = false;
            }
        }
        EV << "Sensor Node Received SRB but Radio was OFF" << endl;
        delete msg;
    }
    else if ( ((std::string) msg->getName()) == "LRB")
    {
        EV << "Sensor Node received LRB" << endl;
        if (radioOn)
        {
            // schedule switch back to low duty cycle
            if (lowDutyCycle)
            {
                // switch to high duty cycle
                lowDutyCycle = false;
                // set timeout
                if(txTimeExpired != nullptr){
                    cancelAndDelete(txTimeoutExpired);
                }
                txTimeoutExpired = new cMessage*("txTimeoutExpired");
                scheduleAt(simTime() + T_off_high, txTimeoutExpired);
            }
        }
        EV << "Sensor Node Received LRB but Radio was OFF" << endl;
        delete msg;
    }
    else if (msg == returnToLowDutyCycle)
    {
        EV << "Return to Low Duty Cycle" << endl;
        lowDutyCycle = true;
    }
    else if (msg == sendData)
    {
        EV << "Sensor Node Sending Data" << endl;
        if (ackLost < 1)
        {
            cModule *c = getModuleByPath("dualBeacon");
            c->par("distinct_pkts_sent_current_passage") = (int) c->par("distinct_pkts_sent_current_passage") + 1;
        }
        // cancel radio-off event
        if(turnRadioOff != nullptr){
            cancelAndDelete(turnRadioOff);
        }
        // send data to sink
        cMessage *dataPacket = new cMessage("dataPacket");
        send(dataPacket, "out");
        // schedule transmission timeout
        if(TxTimeoutExpired != nullptr){
            cancelAndDelete(txTimeoutExpired);
        }
        txTimeoutExpired = new cMessage("txTimeoutExpired");
        scheduleAt(simTime() + txTimeout, txTimeoutExpired);
        // increase counter
        energyTransfer += Ptx * packetDuration;
    }
    else if (msg == txTimeoutExpired)
    {
        EV << "Transmission Timeout" << endl;
        // increase counter
        ackLost++;
        // increase energy counter
        energyTransfer += Prx * txTimeout;
        if (ackLost < 3)
        {
            // retransmit data
            cMessage *dataPacket = new cMessage("dataPacket");
            send(dataPacket,"out");
        }
        else
        {
            // reset counter
            ackLost = 0;
            // return to low duty cycle
            if(returnToLowDutyCycle != nullptr){
                cancelAndDelete(returnToLowDutyCycle);
            }
            returnToLowDutyCycle = new cMessage("returnToLowDutyCycle");
            scheduleAt(simTime(), returnToLowDutyCycle);
            // turn radio off
            if(turnRadioOff != nullptr){
                    cancelAndDelete(turnRadioOff);
            }
            turnRadioOff = new cMessage("turnRadioOff");
            scheduleAt(simTime(), turnRadioOff);
        }
    }
    else if ( ((std::string) msg->getName()) == "ACK")
    {
        EV << "Sensor Node Received ACK" << endl;
        // cancel transmission timeout
        if(txTimeoutExpired != nullptr){
            cancelAndDelete(txTimeoutExpired);
        }
        // reset counter
        ackLost = 0;
        // increase counter
        ackPackets++;
        energyTransfer += Prx * (ackDuration + (2.0 * sigma));
        // schedule new packet transmission
        if(sendData != nullptr){
            cancelAndDelete(sendData);
        }
        sendData = new cMessage("sendData");
        scheduleAt(simTime(), sendData);
        delete msg;
    }
}
void SensorNode2BD::computeTimeouts(){

}
void SensorNode2BD::changeRadioState(bool state){
    radioOn = state;
}
void SensorNode2BD::setInitialRadioState(){
   // get uniform random variable to randomly set initial radio state
   double t = uniform(0, T_on + T_off_low);
   EV << " t is " << t << " and T_on is " << T_on << endl;
       if (t < T_on)
       {
           EV << "Initial Radio State: ON" << endl;
           // initialize radio on
           changeRadioState(true);
           // schedule radio off
           turnRadioOff = new cMessage("turnRadioOff");
           scheduleAt(simTime() + T_on - t, turnRadioOff);
       }
       else
       {
           EV << "Initial Radio State: OFF" << endl;
           // initialize radio to off
           changeRadioState(false);
           // schedule radio on event
           turnRadioOn = new cMessage("turnRadioOn");
           if (lowDutyCycle)
           {
               scheduleAt(simTime() + T_on + T_off_low - t, turnRadioOn);
           }
           else        // high duty cycle
           {
               scheduleAt(simTime() + T_on + T_off_high - t, turnRadioOn);
           }
       }
}
void SensorNode2BD::finish(){
    // print statistics
    EV << "Average Discovery Ratio: " << ((double) timesDiscovered) / ((double) numPassages) * 100.0 << "%" << endl;
    EV << "Average Throughput: " << ((double) ackPackets * packetLength) / ((double) numPassages) << " bytes" << endl;
    EV << "Average Energy Discovery Phase: " << energyDiscovery / ((double) numPassages) * 1000.0 << "mJ" << endl;
    EV << "Average Energy Transfer Phase: " << energyTransfer / ((double) numPassages) * 1000.0 << "mJ" << endl;
}
