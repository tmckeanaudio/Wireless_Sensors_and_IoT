// NED file that includes the necessary Network declaration, Sensor Node, Mobile Sink, and Wireless Channel Modules, 
// and parameters that will be used to complete HW2 that simulates the Stop-And-Wait ARQ protocol

simple SensorNode2BD
{
    parameters:
        @display("i=block/source");
        bool radioOn = default(false);
    	bool lowDutyCycle = default(true);
    	double R; // Discovery Range = 100m
        double r = default(50); // Communication Range = 50m;
    	double T_bi = .1; // TBi = 100ms
    	double T_on = .1; // 100ms Ton period
    	double T_off_low = default(0);
    	double T_off_high = default(0);
    	double deltaLow; // 0.3% low duty cycle
    	double deltaHigh; // 3% high duty cycle 
    	double txTimeout = default(0); 
    	double packetLength = 133; // 133 bytes per packet
    	double sigma = .01; // 10ms
    	int timesDiscovered = default(0);
    	int ackLost = default(0);
    	int ackPackets = default(0);
    	int distinctPacketsSentCurrentPassage = default(0);
    	int numPassages = default(0);
    	int totalPassages = default(1000);
    	double energyDiscovery = default(0);
    	double energyTransfer = default(0);
    	double Prx = default(56.4); // 56.4mW Rx energy
    	double Ptx = default(52.2); // 52.2mW Tx energy
    	double ackDuration = .004; // 4ms ack Duration
    	double packetDuration = .004; // 4ms packet duration
    	double tmpTime = 0.0;
    gates:
        input in;
        output out;
}
simple MobileSinkNode2BD
{
    parameters:
        double T_bi = default(0.1);
        double R; // Discovery Range value
        double speed = 11.111; // Speed is 40Km/hr or 11.11m/s
    	double delta = .001; // Delta is 1ms
    	double x_s = -R + 1; // start X Coordinate
    	double x_e = R + 1; // end X Coordinate
    	double x_c = x_s; // current X Coordinate
    	double y_s = 15;
    	double y_e = 15;
    	double y_c = 15;
    	int correctRx = default(0); 
    	int lastDistinctNoRx = default(-1);
    	int distinctPacketsSentCurrentPassage = default(0);
        @display("i=block/sink");
    gates:
        input in;
        output out;
}
simple WirelessChannel
{
    parameters:
        @display("i=block/network2");
        double p = 0.0; // Message Loss Probability
        double d = 0.0; // Euclidean Distance between MS and SN
    	double x_sn = 0; // X coordinate of SN = 0
    	double y_sn = 0; // Y coordinate of SN = 0
    	double x_ms = -201; // X coordinate of MS
    	double y_ms = 15; // Y coordinate of MS
    	double R; // Discovery range radius
	    double r = 50; // Communication range radius
    gates:
        input in_SN;
        input in_MS;
        output out_SN;
        output out_MS;
}
network dualBeacon
{
    parameters:
        double x_sn = 0; 
        double y_sn = 0;
        double x_ms = default(-201);
        double y_ms = default(15);
        double deltaLow; // Delta Low 0.3%
        double deltaHigh; // Delta High 3%
        double R; // Discovery Range = 100m
        double r = default(50); // Communication Range = 50m;
        int timesDiscovered = 0;
        int numPassages = 0;
        int totalPassages = 1000;
        int ackPackets = 0;
        double energyDiscovery = 0;
        double energyTransfer = 0;
        bool in_discovery_phase = false;
        bool in_communication_phase = false;
        @display("bgb=642,464");
    submodules:
        SN: SensorNode2BD {
            parameters:
                @display("i=,silver;p= 311,224;r=200");
        }
        MS: MobileSinkNode2BD {
            parameters:
                @display("i=,gold;p=80,194");
        }
        WC: WirelessChannel {
            parameters:
                @display("p=211,213");
        }
    connections:
        WC.in_MS <-- MS.out;
        WC.out_MS --> MS.in;
        WC.in_SN <-- SN.out;
        WC.out_SN --> SN.in;
}
