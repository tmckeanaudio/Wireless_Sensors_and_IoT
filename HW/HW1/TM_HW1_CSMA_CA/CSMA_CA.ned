// NED file that includes the necessary Network declaration, Source & Sink Modules, and parameters
// that will be used to complete HW1 that simulates an Unslotted CSMA/CA Algorithm

simple SensorNodeCSMACA
{
    parameters:
        @display("i=block/source");
    int macMinBE;
    int macMaxBE;
    int macMaxCSMABackoffs;
    int totalPackets = default(1000);
    int packets2send = default(1000);
    double packetCreationTime = default(0);
    double Ptx = default(49.5);
    double Prx = default(56.4);
    double Dp = 4.256;
    double D_bp = 0.00032;
    gates:
        output out;
}
simple SinkNodeCSMACA
{
    parameters:
        @display("i=block/sink");
    gates:
        input in[];
}

network CSMA_CA
{
    parameters:
        int numNodes = default(1);
        int concurrentTransmissions = default(0);
        int numDroppedPackets = default(0);
        int numTxPackets = default(0);
        double latency = default(0);
        double energy = default(0);
        bool channelFree = default(true);
        double Dp = 0.004256;
        @display("bgb=624.69336,310.08002");
    submodules:
        source[numNodes]: SensorNodeCSMACA {
            parameters:
                @display("i=,silver");
        }
        sink: SinkNodeCSMACA {
            parameters:
                @display("i=,gold");
        }
    connections:
        for k=0..numNodes-1 {
            source[k].out --> sink.in++;
        }
}
