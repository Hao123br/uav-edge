#include "ns3/core-module.h"
#include "ns3/log.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/applications-module.h"
#include "ns3/lte-module.h"
#include <ns3/buildings-helper.h>
#include "ns3/mobility-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"
#include "ns3/evalvid-client-server-helper.h"
#include "ns3/evalvid-client.h"
#include "ns3/evalvid-server.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("uav-edge");

double UAVtxpower = 15;
std::string ns3_dir;

bool IsTopLevelSourceDir (std::string path)
{
	bool haveVersion = false;
	bool haveLicense = false;

	//
	// If there's a file named VERSION and a file named LICENSE in this
	// directory, we assume it's our top level source directory.
	//

	std::list<std::string> files = SystemPath::ReadFiles (path);
	for (std::list<std::string>::const_iterator i = files.begin (); i != files.end (); ++i)
	{
		if (*i == "VERSION")
		{
			haveVersion = true;
		}
		else if (*i == "LICENSE")
		{
			haveLicense = true;
		}
	}

	return haveVersion && haveLicense;
}

std::string GetTopLevelSourceDir (void)
{
	std::string self = SystemPath::FindSelfDirectory ();
	std::list<std::string> elements = SystemPath::Split (self);
	while (!elements.empty ())
	{
		std::string path = SystemPath::Join (elements.begin (), elements.end ());
		if (IsTopLevelSourceDir (path))
		{
			return path + "/";
		}
		elements.pop_back ();
	}
	NS_FATAL_ERROR ("Could not find source directory from self=" << self);
}

Ptr<Node> config_LTE(Ptr<LteHelper> lte, Ptr<PointToPointEpcHelper> epc)
{
	lte->SetEpcHelper(epc);
	Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(320));

	NS_LOG_UNCOND("Pathloss model: HybridBuildingsPropagationLossModel ");
	Config::SetDefault("ns3::ItuR1411NlosOverRooftopPropagationLossModel::StreetsOrientation", DoubleValue (10));
	lte->SetAttribute ("PathlossModel", StringValue ("ns3::HybridBuildingsPropagationLossModel"));
	lte->SetPathlossModelAttribute ("Frequency", DoubleValue (2.0e9));
	lte->SetPathlossModelAttribute ("ShadowSigmaExtWalls", DoubleValue (0));
	lte->SetPathlossModelAttribute ("ShadowSigmaOutdoor", DoubleValue (3.0));
	lte->SetPathlossModelAttribute ("Los2NlosThr", DoubleValue (100));

	return epc->GetPgwNode();
}

void install_LTE(Ptr<LteHelper> lteHelper, Ptr<PointToPointEpcHelper> epcHelper, NodeContainer UAVs, NodeContainer UEs)
{
	NetDeviceContainer enbDevs;
	NetDeviceContainer ueDevs;

	lteHelper->SetEnbDeviceAttribute ("DlBandwidth", UintegerValue (25)); //Set Download BandWidth
	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (25)); //Set Upload Bandwidth
	enbDevs = lteHelper->InstallEnbDevice(UAVs);

	ueDevs = lteHelper->InstallUeDevice(UEs);

	Ipv4InterfaceContainer ueIpIface;
	ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));
	lteHelper->Attach(ueDevs);

	Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(UAVtxpower));
	Config::SetDefault("ns3::LteEnbPhy::NoiseFigure", DoubleValue(9)); // Default 5
	// Modo de transmissão (SISO [0], MIMO [1])
	Config::SetDefault("ns3::LteEnbRrc::DefaultTransmissionMode", UintegerValue(0));
}

void install_mobility(NodeContainer staticNodes,NodeContainer UAVs, NodeContainer UEs)
{
	Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
	allocator->Add (Vector(500, 500, 0));
	allocator->Add (Vector(500, 520, 0));
	allocator->Add (Vector(500, 540, 0));

	MobilityHelper staticNodesHelper;
	staticNodesHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	staticNodesHelper.SetPositionAllocator (allocator);
	staticNodesHelper.Install(staticNodes);

	MobilityHelper UAVHelper;
	UAVHelper.SetMobilityModel("ns3::WaypointMobilityModel",
								"InitialPositionIsWaypoint", BooleanValue (true));
	UAVHelper.SetPositionAllocator ("ns3::GridPositionAllocator",
								   "MinX", DoubleValue (470.0),
								   "MinY", DoubleValue (470.0),
								   "Z", DoubleValue (0.2),
								   "DeltaX", DoubleValue (20.0),
								   "DeltaY", DoubleValue (20.0),
								   "GridWidth", UintegerValue (2),
								   "LayoutType", StringValue ("RowFirst"));
	UAVHelper.Install (UAVs);
	BuildingsHelper::Install(UAVs);

	MobilityHelper UEHelper;
	UEHelper.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
							   "Mode", StringValue ("Time"),
							   "Time", StringValue ("5s"),
							   "Speed", StringValue ("ns3::UniformRandomVariable[Min=2.0|Max=8.0]"),
							   "Bounds", StringValue ("0|1000|0|1000"));
	UEHelper.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
								  "X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"),
								  "Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"),
								  "Z", StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=1.5]"));
	UEHelper.Install (UEs);
	BuildingsHelper::Install(UEs);
}

void request_video(Ptr<Node> sender_node, Ptr<Node> receiver_node)
{
	static uint16_t m_port = 2000;
	static int request_id = 0;

	Ptr<Ipv4> ipv4 = sender_node->GetObject<Ipv4>();
	Ipv4InterfaceAddress iaddr = ipv4->GetAddress(1, 0);
	Ipv4Address ipAddr = iaddr.GetLocal();

	EvalvidServerHelper server(m_port);
	server.SetAttribute("SenderTraceFilename", StringValue(ns3_dir + std::string("src/evalvid/st_highway_cif.st")));
	//server.SetAttribute("SenderDumpFilename", StringValue("evalvid_sd_" + std::to_string(request_id)));
	server.SetAttribute("SenderDumpFilename", StringValue("evalvid-logs/sd_" + std::to_string(request_id)));
	server.SetAttribute("PacketPayload", UintegerValue(512));
	ApplicationContainer apps = server.Install(sender_node);
	apps.Start(Seconds(5));

	EvalvidClientHelper client(ipAddr, m_port);
	//client.SetAttribute("ReceiverDumpFilename", StringValue("evalvid_rd_" + std::to_string(request_id)));
	client.SetAttribute("ReceiverDumpFilename", StringValue("evalvid-logs/rd_" + std::to_string(request_id)));
	apps = client.Install(receiver_node);
	apps.Start(Seconds(5));

	request_id++;
	m_port++;
}

void NetworkStatsMonitor(FlowMonitorHelper* fmhelper, Ptr<FlowMonitor> flowMon)
{
    flowMon->CheckForLostPackets();
    uint32_t LostPacketsum = 0;
	float PDR, PLR, Delay, Jitter, Throughput;
    std::map<FlowId, FlowMonitor::FlowStats> flowStats = flowMon->GetFlowStats();
    Ptr<Ipv4FlowClassifier> classing = DynamicCast<Ipv4FlowClassifier>(fmhelper->GetClassifier());
	std::ofstream qos_file;
	qos_file.open("qos.txt", std::ofstream::out | std::ofstream::trunc);

    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator stats = flowStats.begin(); stats != flowStats.end(); ++stats) {
        Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow(stats->first);
		PDR = (100 * stats->second.rxPackets) / (stats->second.txPackets);
        LostPacketsum = (stats->second.txPackets) - (stats->second.rxPackets);
		PLR = ((LostPacketsum * 100) / stats->second.txPackets);
		Delay = (stats->second.delaySum.GetSeconds()) / (stats->second.rxPackets);
		Jitter = stats->second.jitterSum.GetSeconds() / (stats->second.rxPackets);
		Throughput = stats->second.rxBytes * 8.0 / (stats->second.timeLastRxPacket.GetSeconds() - stats->second.timeFirstTxPacket.GetSeconds()) / 1024 / 1024;

        std::cout << "Flow ID			: " << stats->first << " ; " << fiveTuple.sourceAddress << " -----> " << fiveTuple.destinationAddress << std::endl;
        std::cout << "Tx Packets = " << stats->second.txPackets << std::endl;
        std::cout << "Rx Packets = " << stats->second.rxPackets << std::endl;
        std::cout << "Lost Packets = " << (stats->second.txPackets) - (stats->second.rxPackets) << std::endl;
        std::cout << "Packets Delivery Ratio (PDR) = " << PDR << "%" << std::endl;
        std::cout << "Packets Lost Ratio (PLR) = " << PLR << "%" << std::endl;
        std::cout << "Delay = " << Delay << " Seconds" << std::endl;
		std::cout << "Jitter = " << Jitter << " Seconds" << std::endl;
        std::cout << "Total Duration		: " << stats->second.timeLastRxPacket.GetSeconds() - stats->second.timeFirstTxPacket.GetSeconds() << " Seconds" << std::endl;
        std::cout << "Last Received Packet	: " << stats->second.timeLastRxPacket.GetSeconds() << " Seconds" << std::endl;
        std::cout << "Throughput: " << Throughput << " Mbps" << std::endl;
        std::cout << "---------------------------------------------------------------------------" << std::endl;
		qos_file << fiveTuple.sourceAddress << " --> " << fiveTuple.destinationAddress << "," << PDR << "," << PLR << "," << Delay << "," << Jitter << "," << Throughput << "\n";
    }

	qos_file.close();
}

void move_uav(Ptr<Node> uav, Vector destination, double n_vel) {
	// get mobility model for uav
    Ptr<WaypointMobilityModel> mob = uav->GetObject<WaypointMobilityModel>();
    Vector m_position = mob->GetPosition();
    double distance;

	distance = CalculateDistance(destination, m_position);
	// 1 meter of accuracy is acceptable
	if(distance <= 1)
		return;

	unsigned int nodeId = uav->GetId();
	double currentTime = Simulator::Now().GetSeconds();
	double nWaypointTime;
	NS_LOG_DEBUG("moving uav with nodeId: " << nodeId << " from " << m_position << " to " << destination << " time: " << currentTime);

	mob->EndMobility();
	mob->AddWaypoint(Waypoint(Simulator::Now(), m_position));

	nWaypointTime = distance/n_vel + currentTime;
	mob->AddWaypoint(Waypoint(Seconds(nWaypointTime), destination));
}

int main(int argc, char* argv[])
{
	uint32_t seed = 4242;
	short simTime = 100;
	short numUAVs = 8;
	short numUEs = 50;

	LogComponentEnable ("uav-edge", LOG_LEVEL_INFO);
	LogComponentEnable ("EvalvidClient", LOG_LEVEL_INFO);
	LogComponentEnable ("EvalvidServer", LOG_LEVEL_INFO);

	CommandLine cmd;
	
	cmd.AddValue("numUAVs", "how many UAVs are in the simulation", numUAVs);
	cmd.AddValue("numUEs", "how many UEs are in the simulation", numUEs);
	cmd.AddValue("seed", "random seed value.", seed);
	cmd.Parse(argc, argv);

	ns3::RngSeedManager::SetSeed(seed);
	ns3_dir = GetTopLevelSourceDir();

	Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
	Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();

	Ptr<Node> pgw = config_LTE(lteHelper, epcHelper);

	NodeContainer remoteHostContainer;
	remoteHostContainer.Create(1);
	Ptr<Node> remoteHost = remoteHostContainer.Get(0);

	NodeContainer UAVs;
	NodeContainer UENodes;

	UAVs.Create(numUAVs);
	UENodes.Create(numUEs);

	InternetStackHelper internet;
	internet.Install(remoteHostContainer);
    internet.Install(UENodes);

	PointToPointHelper p2ph;
	p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
	p2ph.SetDeviceAttribute("Mtu", UintegerValue(1400));
	p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
	NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

	Ipv4AddressHelper ipv4h;
	ipv4h.SetBase("1.0.0.0", "255.0.0.0");
	Ipv4InterfaceContainer internetIpIfaces;
	internetIpIfaces = ipv4h.Assign(internetDevices);

	Ipv4StaticRoutingHelper ipv4RoutingHelper;

	Ipv4Address remoteHostAddr;
    remoteHostAddr = internetIpIfaces.GetAddress(1);
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
	for (uint32_t u = 0; u < UENodes.GetN(); ++u) {
		Ptr<Node> ueNode = UENodes.Get(u);
		Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
		ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
	}

	NodeContainer staticNodes;
	staticNodes.Add(epcHelper->GetSgwNode());
	staticNodes.Add(pgw);
	staticNodes.Add(remoteHost);
	install_mobility(staticNodes, UAVs, UENodes);

	install_LTE(lteHelper, epcHelper, UAVs, UENodes);

	for (uint32_t i = 0; i < UENodes.GetN(); ++i){
		request_video(remoteHost, UENodes.Get(i));
	}

	AnimationInterface animator("lte.xml");
	animator.SetMobilityPollInterval(Seconds(1));
    for (uint32_t i = 0; i < UAVs.GetN(); ++i) {
		animator.UpdateNodeDescription(UAVs.Get(i), "UAV " + std::to_string(i));
		animator.UpdateNodeColor(UAVs.Get(i), 250, 200, 45);
		animator.UpdateNodeSize(UAVs.Get(i)->GetId(),10,10); // to change the node size in the animation.
    }
    for (uint32_t j = 0; j < UENodes.GetN(); ++j) {
		animator.UpdateNodeDescription(UENodes.Get(j), "UE " + std::to_string(j+1));
		animator.UpdateNodeColor(UENodes.Get(j), 20, 10, 145);
		animator.UpdateNodeSize(UENodes.Get(j)->GetId(),10,10);
    }
    animator.UpdateNodeDescription(staticNodes.Get(0), "SGW");
    animator.UpdateNodeDescription(staticNodes.Get(1), "PGW");
    animator.UpdateNodeDescription(staticNodes.Get(2), "RemoteHost");
    for (uint32_t k = 0; k < staticNodes.GetN(); ++k) {
		animator.UpdateNodeColor(staticNodes.Get(k), 110, 150, 45);
		animator.UpdateNodeSize(staticNodes.Get(k)->GetId(),10,10);
    }

	Ptr<FlowMonitor> flowMonitor;
	FlowMonitorHelper flowHelper;
	flowHelper.Install(remoteHost);
	flowMonitor = flowHelper.Install(UENodes);
	Simulator::Schedule(Seconds(simTime-0.001), NetworkStatsMonitor, &flowHelper, flowMonitor);

	move_uav(UAVs.Get(0), Vector(100,100,0.2), 10);
	move_uav(UAVs.Get(1), Vector(900,100,0.2), 10);
	move_uav(UAVs.Get(2), Vector(100,900,0.2), 10);
	move_uav(UAVs.Get(3), Vector(900,900,0.2), 10);

	Simulator::Stop(Seconds(simTime));
	Simulator::Run();
	Simulator::Destroy();
	return 0;
}
