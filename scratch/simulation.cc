#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <time.h> /* time */
#include <unordered_map>
#include <utility> // std::pair
#include <vector>

#include <boost/algorithm/string/classification.hpp> // Include boost::for is_any_of
#include <boost/algorithm/string/split.hpp>          // Include for boost::split

#include "ns3/core-module.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-address.h"
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

#include "ns3/basic-energy-source.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/energy-source-container.h"

#include "ns3/evalvid-client-server-helper.h"
#include "ns3/evalvid-client.h"
#include "ns3/evalvid-server.h"

#include "matriz.h"
#include "ns3/vector.h"

using namespace ns3;

std::default_random_engine generator;

NS_LOG_COMPONENT_DEFINE("uav-edge");

enum class EnergyType { mobility, comms, compute};

class UAVEnergyTrace {
public:
	std::ofstream mobility;
	std::ofstream comms;
	std::ofstream compute;

	UAVEnergyTrace()
	{
		mobility.open("mobility-energy.txt", std::ofstream::out | std::ofstream::trunc);
		comms.open("comms-energy.txt", std::ofstream::out | std::ofstream::trunc);
		compute.open("compute-energy.txt", std::ofstream::out | std::ofstream::trunc);
	}

	~UAVEnergyTrace()
	{
		mobility.close();
		comms.close();
		compute.close();
	}
};

class UAVInfo {
public:
	EventId compute_energy_event;

	UAVInfo(){}
};

// struct that contains info a  about the handovers performed in the network
struct Handover
{
  double time;
  int user;
  int source;
  int target;

  Handover(double t, int u, int s, int tg)
      : time{t}, user{u}, source{s}, target{tg} {}

  // operator to compare two handover instances within a given time window
  bool operator==(const Handover &other) const
  {
    return std::abs(other.time - time) < 2 && user == other.user;
    // return user == other.user  && time - other.time > 0;
    //&& source == other.source && source == other.source &&
    //    target == other.target;
  }

  friend std::ostream &operator<<(std::ostream &os, const Handover &h)
  {
    os << "Handover(" << h.time << ", " << h.user << ", " << h.source << ", "
       << h.target << ")";

    return os;
  }
};

const float MOBILITY_ENERGY_INTERVAL = 1; //second
const float COMMS_ENERGY_INTERVAL = 1; //seconds
const float COMPUTE_ENERGY_INTERVAL = 1; //seconds
const float COMMS_ENERGY_COST = 50; //Joules
const float COMPUTE_ENERGY_COST = 30; //Joules
const unsigned int ENB_BW = 25;
const unsigned int UAV_BW = 25;

uint32_t numStaticCells = 20;
uint32_t numUAVs = 8;
uint32_t numUEs = 50;
// these variables must be initialized in main()
//---------------------------//
uint32_t numBSs;
uint32_t numEdgeServers;
uint32_t number_of_hot_spots;
//---------------------------//
double ENBtxpower = 46;
double UAVtxpower = 20;
float uav_speed = 10;
std::string ns3_dir;
UAVEnergyTrace uav_energy_trace;
unordered_map<unsigned int, Ptr<Node>> uav_by_id;
unordered_map<unsigned int, UAVInfo> uavs_info;

// scenario variables
std::vector<double> ues_sinr;
std::ofstream ues_sinr_file;
std::vector<double> time_to_centroid;
std::ofstream time_to_centroid_file;
std::ofstream ue_positions_log;
uint32_t active_drones = 0;
// std::string clustering_algoritm = "kmeans";

Time management_interval = Seconds(1);
// amount of times manager is called
int monitor_calls = 0;

std::string mobil_trace = "traces/koln.tcl";
std::string req_mode = "random";
std::string requests_trace = "traces/requests.tcl_but_not_really";
std::string handover_policy = "iuavbs";
float distance_multiplier = 1.0 / 10;

uint16_t node_remote = 1; // HOST_REMOTO
// double cell_thr_average = 0; // average throughput of used cells

/* ================= control variables ==================*/
bool disableDl = false;
bool disableUl = false;
bool enablePrediction = true;
bool verbose = false;
bool enableHandover = true;
bool useCa = false;

/*============= state variables =======================*/
/* connection management structures */
matriz<int> cellUe;
std::vector<std::vector<double>> neighbors;
// structure to store handover predictions
// // [0] -> time of the handover
// // [1] -> source cell
// // [2] -> target cell
matriz<int> handoverPredictions;
std::vector<double> cell_usage;
std::vector<double> cell_throughput;

std::vector<Ipv4Address> user_ip;
std::vector<double> user_requests;
std::vector<double> user_throughput;
std::vector<bool> unserved_users;

std::vector<std::vector<int>> edgeUe;
std::vector<std::vector<int>> edgeMigrationChart;
matriz<Ipv4Address> x2AddressMatrix;

std::map<int, int> handovers_per_second;

// peek results at the end of the simulation
std::vector<double> peek_user_throughput;
std::vector<double> peek_cell_throughput;
std::vector<double> peek_service_level;
std::vector<double> peek_uav_usage;

// store historical handover info
std::vector<Handover> handover_vector;
unsigned int handNumber = 0; // number of handovers executed

std::vector<bool> drones_in_use(numUAVs);
std::vector<bool> hot_spots_served(number_of_hot_spots);

std::map<int, std::map<int, int>> rnti_cells;

// mapping nodelist ids to actual imsis
std::map<int, int> path_imsi;

// /*MIGRATION VARIABLES*/
// enable logs
// perform migrations
bool doMigrate = true;
Time managerInterval = Seconds(1);
std::string algorithm = "iuavbs";

// server characteristics
// the first index is the metric: lat, bw, and cost
// second index is the type of server: mist, edge, fog, cloud
int serverReqs[4][3] = {{1, 1, 4}, {1, 2, 3}, {10, 10, 2}, {100, 100, 1}};

// applications class 1, 2, 3, and 4
// latency in ms and bw in mbps, and prioritary
int applicationReqs[3][3] = {{1, 10, 1}, {10, 100, 1}, {1000, 1, 0}};
uint16_t applicationType = 0;

//-----VARIABLES THAT DEPEND ON THE NUMBER OF SERVERS----
// The resources variable tells which server has one or
// more of the recources needed in this simulation
// the resources are:
vector<uint16_t> resources;

// type of cell position allocation
bool rowTopology = false;
bool randomCellAlloc = true;

// ----------VARIABLES TO CALCULATE METRICS-----------
std::vector<int> latency;
/* END OF MIGRATION VARIABLES */

std::vector<int> cost;
// node containers as global objects to make mobility management possible
NodeContainer uavNodes;
NodeContainer ueNodes;
NodeContainer BSNodes;
NodeContainer staticBSNodes;

NetDeviceContainer enbDevs;
NetDeviceContainer ueDevs;

/*====== REQUIRED PROTOTYPES =======*/
int get_cell(int);
int get_cell_from_imsi(int);
int get_closest_center_index(Ptr<Node>, std::vector<std::pair<int, int>>);
Vector get_node_position(Ptr<Node>);
void requestApplication(Ptr<Node>, Ptr<Node>, double);
void handoverManager(std::string);
void migrate(Ptr<Node> , Ptr<Node>, Ipv4Address, Ipv4Address);
int getCellId(int);
Ptr<ListPositionAllocator> generatePositionAllocator(int, int, std::string allocation);

// global lte helper for handover management
Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();

void initialize_vectors()
{
	/*============= state variables =======================*/
	/* connection management structures */
	cellUe.setDimensions(numBSs, numUEs);
	neighbors.assign(numBSs, std::vector<double>(numUEs, 0));
	handoverPredictions.setDimensions(numUEs, 3);
	x2AddressMatrix.setDimensions(numBSs, numBSs);

	cell_usage.assign(numBSs, 0);
	cell_throughput.assign(numBSs, 0);

	user_ip.resize(numUEs);
	user_requests.assign(numUEs, 0);
	user_throughput.assign(numUEs, 0);
	unserved_users.assign(numUEs, 0);

	edgeUe.assign(numEdgeServers, std::vector<int>(numUEs, 0));
	edgeMigrationChart.assign(numUEs, std::vector<int>(numEdgeServers, 0));

	drones_in_use.resize(numUAVs);
	hot_spots_served.resize(number_of_hot_spots);

	//-----VARIABLES THAT DEPEND ON THE NUMBER OF SERVERS----
	// The resources variable tells which server has one or
	// more of the recources needed in this simulation
	// the resources are:
	resources.assign(numEdgeServers, 10);
}

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

	Config::SetDefault("ns3::LteEnbPhy::NoiseFigure", DoubleValue(9)); // Default 5
	// Modo de transmissão (SISO [0], MIMO [1])
	Config::SetDefault("ns3::LteEnbRrc::DefaultTransmissionMode", UintegerValue(0));
	return epc->GetPgwNode();
}

void populate_x2_matrix(NodeContainer BSNodes)
{
	std::unordered_map<uint32_t, uint32_t> ip2id;

	for(unsigned int i = 0; i < numBSs; i++)
	{
		auto node = BSNodes.Get(i);
		auto ip = node->GetObject<Ipv4>();

		//interface 2 and above are X2 interfaces
		for(unsigned int j = 2; j < ip->GetNInterfaces(); j++) {
			auto addr = ip->GetAddress(j,0).GetLocal();
			ip2id[addr.Get()] = i;
		}
	}

	for(unsigned int i = 0; i < numBSs; i++)
	{
		auto node = BSNodes.Get(i);
		auto ip = node->GetObject<Ipv4>();
		//interface 2 and above are X2 interfaces
		for(unsigned int j = 2; j < ip->GetNInterfaces(); j++) {
			auto addr = ip->GetAddress(j,0).GetLocal();
			auto mask = ip->GetAddress(j,0).GetMask();
			uint32_t hostAddr = addr.Get() & 0b11;
			uint32_t peerAddr = hostAddr==1 ? 2:1;
			uint32_t peer = addr.CombineMask(mask).Get() + peerAddr;
			uint32_t id = ip2id[peer];
			x2AddressMatrix[i][id] = addr;
		}
	}
}

void install_LTE(Ptr<LteHelper> lteHelper, Ptr<PointToPointEpcHelper> epcHelper, NodeContainer staticBSNodes, NodeContainer uavNodes, NodeContainer UEs)
{
	Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(ENBtxpower));
	lteHelper->SetEnbDeviceAttribute ("DlBandwidth", UintegerValue (ENB_BW)); //Set Download BandWidth
	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (ENB_BW)); //Set Upload Bandwidth
	enbDevs = lteHelper->InstallEnbDevice(staticBSNodes);
	
	Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(UAVtxpower));
	lteHelper->SetEnbDeviceAttribute ("DlBandwidth", UintegerValue (UAV_BW)); //Set Download BandWidth
	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (UAV_BW)); //Set Upload Bandwidth
	enbDevs.Add(lteHelper->InstallEnbDevice(uavNodes));
	lteHelper->AddX2Interface(BSNodes);
	populate_x2_matrix(BSNodes);

	ueDevs = lteHelper->InstallUeDevice(UEs);

	Ipv4InterfaceContainer ueIpIface;
	ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));
	lteHelper->Attach(ueDevs);
}

void move_uav(Ptr<Node> uav, Vector destination, double n_vel);

void install_mobility(NodeContainer staticNodes, NodeContainer staticBSNodes, NodeContainer uavNodes, NodeContainer UEs)
{
	Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
	allocator->Add (Vector(500, 500, 0));
	allocator->Add (Vector(500, 520, 0));
	allocator->Add (Vector(500, 540, 0));

	MobilityHelper staticNodesHelper;
	staticNodesHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	staticNodesHelper.SetPositionAllocator (allocator);
	staticNodesHelper.Install(staticNodes);

	MobilityHelper enbHelper;
	enbHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	auto bsPosition = generatePositionAllocator(numStaticCells, 2000, "random");
	enbHelper.SetPositionAllocator(bsPosition);
	enbHelper.Install(staticBSNodes);
	BuildingsHelper::Install(staticBSNodes);

	MobilityHelper UAVHelper;
	UAVHelper.SetMobilityModel("ns3::WaypointMobilityModel",
								"InitialPositionIsWaypoint", BooleanValue (true));
	auto uavPosition = generatePositionAllocator(numUAVs, 2000, "random");
	UAVHelper.SetPositionAllocator (uavPosition);
	UAVHelper.Install (uavNodes);
	BuildingsHelper::Install(uavNodes);

	Ns2MobilityHelper UEHelper = Ns2MobilityHelper(mobil_trace);
	UEHelper.Install(UEs.Begin(), UEs.End());
	BuildingsHelper::Install(UEs);
}

void installEnergy(NodeContainer uavs){
	/*
	* Create and install energy source on the nodes using helper.
	*/
	// source helper
	BasicEnergySourceHelper basicSourceHelper;
	basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (350e3));
	// set update interval
	basicSourceHelper.Set ("PeriodicEnergyUpdateInterval",
						 TimeValue (Seconds (1.5)));
	// install source
	basicSourceHelper.Install (uavs);
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

void write_energy_trace(EnergyType eType, unsigned int id, double energy_spent, double remaining_energy)
{
	double currentTime = Simulator::Now().GetSeconds();
	UAVEnergyTrace& eTrace = uav_energy_trace;
	std::ostringstream message;

	message << currentTime
			<< " " << id
			<< " " << energy_spent
			<< " " << remaining_energy
			<< "\n";

	switch(eType)
	{
		case EnergyType::mobility:
			eTrace.mobility << message.str();
			break;

		case EnergyType::comms:
			eTrace.comms << message.str();
			break;

		case EnergyType::compute:
			eTrace.compute << message.str();
			break;
	}
}

Ptr<BasicEnergySource> get_energy_source(Ptr<Node> uav){
	Ptr<EnergySourceContainer> energy_container;
	Ptr<BasicEnergySource> source;

	energy_container = uav->GetObject<EnergySourceContainer> ();
	source = DynamicCast<BasicEnergySource> (energy_container->Get(0));

	return source;
}

void update_mobility_energy(NodeContainer uavs)
{
	Ptr<Node> uav;
	unsigned int id;
	Vector pos;
	Ptr<BasicEnergySource> source;
	double energy_spent, remaining_energy;

	for (auto iter = uavs.Begin(); iter != uavs.End(); iter++)
	{
		uav = *iter;

		id = uav->GetId();
		pos = uav->GetObject<MobilityModel>()->GetPosition ();

		source = get_energy_source(uav);

		//Ordem de entrada dos parametros: posição X, posição Y, posição Z, tempo de atualização, velocidade
		energy_spent = source->UpdateEnergyMobSource(pos.x,pos.y,pos.z, MOBILITY_ENERGY_INTERVAL, uav_speed);
		remaining_energy = source->GetRemainingEnergy();

		write_energy_trace(EnergyType::mobility, id, energy_spent, remaining_energy);
	}

	Simulator::Schedule(Seconds(MOBILITY_ENERGY_INTERVAL), &update_mobility_energy, uavs);
}

void update_comms_energy(NodeContainer uavs){
	Ptr<Node> uav;
	unsigned int id;
	Ptr<BasicEnergySource> source;
	double remaining_energy;

	for (auto iter = uavs.Begin(); iter != uavs.End(); iter++)
	{
		uav = *iter;
		id = uav->GetId();
		source = get_energy_source(uav);
		source->ProcessEnergy(COMMS_ENERGY_COST);

		remaining_energy = source->GetRemainingEnergy();
		write_energy_trace(EnergyType::comms, id, COMMS_ENERGY_COST, remaining_energy);
	}

	Simulator::Schedule(Seconds(COMMS_ENERGY_INTERVAL), &update_comms_energy, uavs);
}

void update_compute_energy(Ptr<Node> uav){
	unsigned int id;
	Ptr<BasicEnergySource> source;
	double remaining_energy;

	id = uav->GetId();
	source = get_energy_source(uav);
	source->ProcessEnergy(COMPUTE_ENERGY_COST);

	remaining_energy = source->GetRemainingEnergy();
	write_energy_trace(EnergyType::compute, id, COMPUTE_ENERGY_COST, remaining_energy);

	uavs_info[id].compute_energy_event = Simulator::Schedule(Seconds(COMPUTE_ENERGY_INTERVAL), &update_compute_energy, uav );
}

void initial_compute_energy(NodeContainer uavs){
	for(auto iter = uavs.Begin(); iter != uavs.End(); ++iter)
	{
		update_compute_energy(*iter);
	}
}

std::string exec(std::string cmd)
{
  std::array<char, 128> buffer;
  std::string result;
  std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
  if (!pipe)
    throw std::runtime_error("popen() failed!");
  while (!feof(pipe.get()))
  {
    if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
      result += buffer.data();
  }
  return result;
}

Ptr<ListPositionAllocator>
generatePositionAllocator(int number_of_nodes = 300, int area = 1000,
                          std::string allocation = "random")
{

  Ptr<ListPositionAllocator> HpnPosition =
      CreateObject<ListPositionAllocator>();
  std::uniform_int_distribution<int> distribution(0, area);

  if (allocation == "koln")
  {
    std::ifstream cellList("traces/cellList_koln");
    double a, b, c;
    while (cellList >> a >> b >> c)
    {
      NS_LOG_DEBUG("adding cell to position " << b * distance_multiplier << " "
                                     << c * distance_multiplier);
      HpnPosition->Add(
          Vector3D(b * distance_multiplier, c * distance_multiplier, 45));
    }
  }

  else if (allocation == "random")
  {
    for (int i = 0; i < number_of_nodes; i++)
    {
      HpnPosition->Add(
          Vector3D(distribution(generator), distribution(generator), 45));
    }
  }

  else
  {
    NS_FATAL_ERROR("invalid cell allocation policy.");
  }
  return HpnPosition;
}

std::vector<std::pair<int, int>> create_hot_spots()
{
  std::vector<std::pair<int, int>> centers;
  std::uniform_int_distribution<int> distribution(0, 2000);

  for (uint32_t i = 0; i < number_of_hot_spots; i++)
  {
    std::pair<int, int> center;
    center.first = distribution(generator);
    center.second = distribution(generator);
    NS_LOG_DEBUG("Adding centroid to " << center.first << " " << center.second);
    centers.push_back(center);
  }

  return centers;
}

std::map<std::pair<int, int>, double> populate_requests_trace()
{
  int multiplier = 1024;
  std::map<std::pair<int, int>, double> requests; //

  // at 978 user 99 requests 1043.98848 bytes
  std::ifstream trace_file(requests_trace);
  std::string str;

  while (std::getline(trace_file, str))
  {
    vector<std::string> split_string;
    boost::split(split_string, str, boost::is_any_of(" "));

    // pegar triple time [1], user [], valor da request
    if (verbose)
    {
      NS_LOG_DEBUG(split_string[1] << " " << split_string[3] << " " << split_string[5]
                          << " ");
    }
    int user = stoi(split_string[3]);
    int time = stoi(split_string[1]);
    double request_value = stod(split_string[5]) * multiplier; // save in bytes
    requests[{time, user}] = request_value;
  }
  trace_file.close();
  return requests;
}

void generate_requests(Ptr<Node> remoteHost,
                       std::vector<std::pair<int, int>> centers,
                       std::map<std::pair<int, int>, double> &requests,
                       int max_payload = 10 * 1024 * 1024, int decay = 500)
{

  // zero cell usage so it's updated every second
  for (auto &cu : cell_usage)
    cu = 0;

  if (req_mode == "trace")
  {
    for (uint32_t i = 0; i < ueNodes.GetN(); i++)
    {
      // payload associated with pair of time and user id
      // double payload = requests[{(int)Simulator::Now().GetSeconds(), i}];
      double payload = 1024;
      if (int cell = getCellId(i) != -1)
      {
        cell_usage[cell] += payload;
        user_requests[i] = payload;
        requestApplication(remoteHost, ueNodes.Get(i), payload);
      }
    }
  }

  else if (req_mode == "random")
  {
    // generate payload user-wise
    for (uint32_t i = 0; i < ueNodes.GetN(); i++)
    {
      int serving_node = 0;

      // get distance to closest hot spot and calculate payload
      int index_to_closest = get_closest_center_index(ueNodes.Get(i), centers);

      std::pair<int, int> closest_surge = centers[index_to_closest];
      Vector3D surge(closest_surge.first, closest_surge.second, 1);

      // payload follows an exponential function with max when dist to hot spot
      // =
      // 0
      Vector node_position = get_node_position(ueNodes.Get(i));
      double dist = CalculateDistance(surge, node_position);
      int payload = max_payload * exp((dist * -1) / decay);

      if (payload)
      {
        NS_LOG_DEBUG("requesting app from user " << i << " to server " << serving_node
                                        << " with payload " << payload
                                        << " bytes");

        if (int cell = getCellId(i) != -1)
        {
          cell_usage[cell] += payload;
          user_requests[i] = payload;
          requestApplication(remoteHost, ueNodes.Get(i), payload);
        }
      }
    }
  }

  Simulator::Schedule(management_interval, &generate_requests, remoteHost,
                      centers, requests, 10 * 1024 * 1024, 500);
}

// populate pairing from nodeid to imsi
int populate_path_imsi(std::string path, int imsi)
{
  int nodeid = -1;

  std::vector<std::string> split_path;
  boost::split(split_path, path, boost::is_any_of("/"));
  nodeid = stoi(split_path[2]);

  if (imsi != -1)
    path_imsi[nodeid] = imsi;

  return nodeid;
}

int get_nodeid_from_path(std::string path)
{
  int nodeid;

  std::vector<std::string> split_path;
  boost::split(split_path, path, boost::is_any_of("/"));
  nodeid = stoi(split_path[2]);

  // if key is present, return nodeid which is 1 less than imsi
  if (path_imsi.find(nodeid) != path_imsi.end())
    return path_imsi[nodeid] - 1;
  return -1;
}

// getter methods
Vector get_node_position(Ptr<Node> node)
{
  Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
  return mob->GetPosition();
}

int getCellId(int user_id)
{
  for (uint32_t i = 0; i < numBSs; i++)
  {
    if (cellUe[i][user_id])
    {
      return i;
    }
  }
  return -1;
}

// get node imsi from cellId and rnti
int get_imsi(int cellId, int rnti)
{
  return rnti_cells[cellId][rnti] == 0 ? -1 : rnti_cells[cellId][rnti];
}

int get_cell(int user_id)
{
  for (uint32_t i = 0; i < numBSs; i++)
  {
    if (cellUe[i][user_id])
    {
      return i;
    }
  }
  return -1;
}


int get_cell_from_imsi(int imsi)
{
  int servingCell = 0;
  for (uint32_t i = 0; i < numBSs; i++)
  {
    if (cellUe[i][imsi - 1] != 0)
    {
      servingCell = i;
    }
  }
  return servingCell;
}

bool is_drone(int node_id) { return node_id >= (int)numStaticCells; }

int getNodeId(Ptr<Node> node, string type = "server")
{
  // seleced the desired node container
  NodeContainer tmpNodesContainer;
  if (type == "server" || type == "enb")
    tmpNodesContainer = BSNodes;
  else if (type == "ue")
    tmpNodesContainer = ueNodes;

  // find th enode id
  for (uint32_t i = 0; i < tmpNodesContainer.GetN(); ++i)
  {
    if (node == tmpNodesContainer.Get(i))
    {
      // NS_LOG_UNCOND("node " << node << " is " << tmpNodesContainer.Get(i) <<
      // " ?");
      return i;
    }
  }

  return -1;
}

int getEdge(int nodeId)
{
  int edgeId = -1;
  for (uint32_t i = 0; i < numEdgeServers; ++i)
    if (edgeUe[i][nodeId])
    {
      edgeId = i;
    }
  return edgeId;
}

int get_closest_center_index(Ptr<Node> node,
                             std::vector<std::pair<int, int>> centers)
{
  Vector m_position = get_node_position(node);
  double dist = INT_MAX;
  int closest = -1;

  if (centers.size() == 0)
    return closest;

  for (uint32_t i = 0; i < number_of_hot_spots; i++)
  {
    if (dist > CalculateDistance(m_position, Vector3D(centers[i].first,
                                                      centers[i].second, 1)))
    {
      closest = i;
    }
  }
  return closest;
}

// this is not workiiiing
int get_user_id_from_ipv4(Ipv4Address ip)
{

  for (uint32_t i = 0; i < numUEs; i++)
  {
    if (user_ip[i] == ip)
    {
      return i;
    }
  }
  return -1;
}

void HandoverPrediction(int nodeId, int timeWindow)
{
    std::string mobilityTrace = mobil_trace;

    // means no connection has been found
    // happens if it's called too early in the simulation
    if (getCellId(nodeId) == -1)
        return;

    // receive a nodeId, and a time window, and return if a handover is going to happen in this time window.
    std::ifstream mobilityFile(mobilityTrace);
    NS_ASSERT_MSG(mobilityFile.is_open(), "Error opening prediction file.");

    string nodeColumn;
    string fileLines;

    // coordinate variables
    double node_x, node_y, node_z, node_position_time;
    double shortestDistance = numeric_limits<int>::max();
    int closestCell = numeric_limits<int>::max();

    // tmp veriables to read file
    // node_position_time = time of the position
    string aux1, aux2, aux4, aux5;
    string cell_id;

    while (getline(mobilityFile, fileLines))
    {
        if (fileLines.find("setdest") != string::npos)
        {

            std::stringstream ss(fileLines);
            // cout << ss.str();
            ss >> aux1 >> aux2 >> node_position_time >> aux4 >> aux5 >> node_x >> node_y >> node_z;

            nodeColumn = "\"$node_(" + to_string(nodeId) + ")";
            // cout << "nodeColumn" << nodeColumn << "\n";
            // cout << "aux: " << aux4 << "\n";
            // cin.get();

            // for (int time_offset = 0; time_offset < timeWindow; time_offset++)
            if (aux4 == nodeColumn && Simulator::Now().GetSeconds() + timeWindow == round(node_position_time))
            {
                Vector uePos = Vector(node_x, node_y, node_z);

                // double distanceServingCell = CalculateDistance(uePos, enbNodes.Get(getCellId(nodeId))->GetObject<MobilityModel>()->GetPosition ());

                // calculate distance from node to each enb
                for (unsigned int i = 0; i < numBSs; ++i)
                {
                    // get Ith enb  position
                    Vector enbPos = BSNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
                    // get distance
                    double distanceUeEnb = CalculateDistance(uePos, enbPos);

                    // get closest enb
                    if (distanceUeEnb < shortestDistance)
                    {
                        closestCell = i;
                        shortestDistance = distanceUeEnb;
                    }
                }

                // if closest enb != current, predict handover
                if (closestCell != getCellId(nodeId))
                {
                    std::cout << "Handover to happen at " << node_position_time << endl;
                    std::cout << "Node " << nodeId << " from cell " << getCellId(nodeId) << " to cell " << closestCell << endl;
                    handoverPredictions[nodeId][0] = node_position_time;
                    handoverPredictions[nodeId][1] = getCellId(nodeId);
                    handoverPredictions[nodeId][2] = closestCell;
                }
            }
        }
    }

    mobilityFile.close();
}

// ================ EVENT LISTENERS ====================
void ReportUeMeasurementsCallback(std::string path, uint16_t rnti,
                                  uint16_t cellId, double rsrp, double rsrq,
                                  bool servingCell, uint8_t componentCarrierId)

{
  int imsi = get_imsi(cellId, rnti);

  int node_id = populate_path_imsi(path, imsi);

  if (verbose)
  {
    NS_LOG_DEBUG("Simulation time: " << Simulator::Now().GetSeconds());
    NS_LOG_DEBUG(path);
    NS_LOG_DEBUG("rnti " << rnti);
    NS_LOG_DEBUG("cellid " << cellId);
    NS_LOG_DEBUG("rsrp " << rsrp);
    NS_LOG_DEBUG("rsrq " << rsrq);
    NS_LOG_DEBUG("imsi " << imsi);
    NS_LOG_DEBUG("path imsi " << path_imsi[node_id]);
    NS_LOG_DEBUG("serving cell " << servingCell);
    NS_LOG_DEBUG("cc id " << (int)componentCarrierId);
    NS_LOG_DEBUG("\n");
  }
  // store all received signals here, must define a signal threhold to ignore
  // cells that are no longer reachable
  if (path_imsi[node_id] != 0)
  {
    neighbors[cellId - 1][path_imsi[node_id] - 1] = rsrp;
  }
  // call handover manager upon receiving new measurements
  handoverManager(path);
}

void RecvMeasurementReportCallback(std::string path, uint64_t imsi,
                                   uint16_t cellId, uint16_t rnti,
                                   LteRrcSap::MeasurementReport meas)
{
  if (verbose)
  {
    NS_LOG_DEBUG("Simulation time: " << Simulator::Now().GetSeconds());
    NS_LOG_DEBUG(path);
    NS_LOG_DEBUG(imsi);
    NS_LOG_DEBUG(cellId);
    NS_LOG_DEBUG(rnti);
    NS_LOG_DEBUG((int)meas.measResults.measId);
    NS_LOG_DEBUG("\n");
  }
}

void NotifyConnectionEstablishedUe(string context,
                                   uint64_t imsi,
                                   uint16_t cellid,
                                   uint16_t rnti)
{
    NS_LOG_INFO(Simulator::Now().GetSeconds() << " " << context << " UE IMSI " << imsi << ": connected to CellId " << cellid << " with RNTI " << rnti << "\n");

    std::stringstream temp_cell_dir;
    std::stringstream ueId;
    temp_cell_dir << "./v2x_temp/" << cellid;
    ueId << temp_cell_dir.str() << "/" << rnti;
    if (mkdir(temp_cell_dir.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0)
    {
    }
    ofstream outfile(ueId.str().c_str());
    outfile << imsi << endl;
    outfile.close();

    /*
TRY TO CONNECT TO THE EDGE SERVER IN THE BEGGINING OF THE SIMULATION,
IF NOT POSSIBLE ASSIGN A RANDOM FOG SERVER TO THE USER.
*/
    // resources[cellid - 1] => IS THE POSITION OF THE CELL'S EDGE
    if (resources[cellid - 1] == 0)
    {
        int fallbackServer;
        // iterate untill an edge with available resources has been chosen
        do
        {
            fallbackServer = rand() % (numEdgeServers);
            NS_LOG_UNCOND("fallbackServer " << fallbackServer);
            // make while condition true to reiterate
            if (resources[fallbackServer] == 0)
                fallbackServer = cellid - 1;
        } while (fallbackServer == cellid - 1);

        std::cout << "Failed to allocate user" << imsi << " in edge " << cellid - 1 << "\n";
        std::cout << "allocating to random fog " << fallbackServer << endl;
        resources[fallbackServer]--;
        edgeUe[fallbackServer][imsi - 1] = 1;
    }
    else
    {
        std::cout << "User " << imsi << " connected to edge " << cellid - 1 << endl;
        edgeUe[cellid - 1][imsi - 1] = 1;
        resources[cellid - 1]--;
    }

    cellUe[cellid - 1][imsi - 1] = rnti;
}

void NotifyHandoverStartUe(string context,
                           uint64_t imsi,
                           uint16_t cellid,
                           uint16_t rnti,
                           uint16_t targetCellId)
{
    std::cout << Simulator::Now().GetSeconds() << " " << context << " UE IMSI " << imsi << ": previously connected to CellId " << cellid << " with RNTI " << rnti << ", doing handover to CellId " << targetCellId << "\n";

    cellUe[cellid - 1][imsi - 1] = 0;

    if (algorithm == "greedy")
    {
        // migration of handover start
        Simulator::Schedule(Simulator::Now(), &migrate,
				BSNodes.Get(cellid - 1),
				BSNodes.Get(targetCellId - 1),
				x2AddressMatrix[cellid - 1][targetCellId - 1],
				x2AddressMatrix[targetCellId - 1][cellid - 1]);
        edgeUe[cellid - 1][imsi - 1] = 0;
        edgeUe[targetCellId - 1][imsi - 1] = 1;
    }

    ++handNumber;
}

void NotifyHandoverEndOkUe(string context,
                           uint64_t imsi,
                           uint16_t cellid,
                           uint16_t rnti)
{
    std::cout << Simulator::Now().GetSeconds() << " " << context << " UE IMSI " << imsi << ": successful handover to CellId " << cellid << " with RNTI " << rnti << "\n";

    std::stringstream target_cell_dir;
    std::stringstream newUeId;
    target_cell_dir << "./v2x_temp/" << cellid;
    newUeId << target_cell_dir.str() << "/" << rnti;
    if (mkdir(target_cell_dir.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0)
    {
    }
    ofstream outfile(newUeId.str().c_str());
    outfile << imsi << endl;
    outfile.close();

    cellUe[cellid - 1][imsi - 1] = rnti;
}

void NotifyConnectionEstablishedEnb(string context,
                                    uint64_t imsi,
                                    uint16_t cellid,
                                    uint16_t rnti)
{
    std::cout << Simulator::Now().GetSeconds() << " " << context << " eNB CellId " << cellid << ": successful connection of UE with IMSI " << imsi << " RNTI " << rnti << "\n";
}

void NotifyHandoverStartEnb(string context,
                            uint64_t imsi,
                            uint16_t cellid,
                            uint16_t rnti,
                            uint16_t targetCellId)
{
    std::cout << Simulator::Now().GetSeconds() << " " << context << " eNB CellId " << cellid << ": start handover of UE with IMSI " << imsi << " RNTI " << rnti << " to CellId " << targetCellId << "\n";
}

void NotifyHandoverEndOkEnb(string context,
                            uint64_t imsi,
                            uint16_t cellid,
                            uint16_t rnti)
{
    std::cout << Simulator::Now().GetSeconds() << " " << context << " eNB CellId " << cellid << ": completed handover of UE with IMSI " << imsi << " RNTI " << rnti << "\n";
}

void PhySyncDetectionCallback(std::string context, uint64_t imsi, uint16_t rnti,
                              uint16_t cellId, std::string type,
                              uint8_t count)
{
  NS_LOG_DEBUG("PhySyncDetectionCallback imsi " << imsi << " cellid " << cellId
                                       << " rnti " << rnti);
}

void RadioLinkFailureCallback(std::string context, uint64_t imsi,
                              uint16_t cellId, uint16_t rnti)
{
  NS_LOG_DEBUG("RadioLinkFailur eCallback " << imsi << " cellid " << cellId << " rnti "
                                   << rnti);
}

/* ======================= TRAFFIC GENERATORS ===============*/

void migrate(Ptr<Node> sourceServer, Ptr<Node> targetServer,
             Ipv4Address sourceServerAddress, Ipv4Address targetServerAddress)
{
  static int migrationPort = 10000;
  // return if migration is not available
  if (!doMigrate)
  {
    std::cout << "Migration not enabled. :(\n";
    // return;
  }

  if (resources[getNodeId(targetServer)] <= 0)
  {
    NS_LOG_UNCOND("MIGRATION FAILED DUE TO LACK OF RESOURCES");
    return;
  }
  if (getNodeId(targetServer) < 0)
    return;

  NS_LOG_UNCOND("Migration from node " << getNodeId(sourceServer) << " to node "
                                       << getNodeId(targetServer));

  resources[getNodeId(sourceServer)]++;
  resources[getNodeId(targetServer)]--;

  // stop before any traffic is actually sent
  // do this to speed up the simulation
  return;

  // cout << "Starting migration from node " << sourceServerAddress << " to node
  // " << targetServerAddress << ".\n";
  ++migrationPort;
  UdpServerHelper server(migrationPort);
  ApplicationContainer apps = server.Install(targetServer);
  apps.Start(Simulator::Now());
  // apps.Stop (Simulator::Now()+Seconds(5));

  //
  // Create one UdpClient application to send UDP datagrams from node zero to
  // node one.
  //

  uint32_t MaxPacketSize = 1024;
  // uint32_t maxPacketCount = migrationSize / MaxPacketSize;
  uint32_t maxPacketCount = 10000;
  // tyr to migrate this in 10 senconds at most
  Time interPacketInterval = MilliSeconds(1);
  UdpClientHelper client(targetServerAddress, migrationPort);
  client.SetAttribute("MaxPackets", UintegerValue(maxPacketCount));
  client.SetAttribute("Interval", TimeValue(interPacketInterval));
  client.SetAttribute("PacketSize", UintegerValue(MaxPacketSize));
  apps = client.Install(sourceServer);
  apps.Start(Simulator::Now());
}

void requestApplication(Ptr<Node> ueNode, Ptr<Node> targetServer,
                        double payload = 0)
{

  // use this snippet here
  DataRateValue dataRateValue = DataRate("1Mbps");
  uint64_t bitRate = dataRateValue.Get().GetBitRate();
  uint32_t packetSize = 1024; // bytes
  NS_LOG_DEBUG("bit rate " << bitRate);
  double interPacketInterval = static_cast<double>(packetSize * 8) / bitRate;
  Time udpInterval = Seconds(interPacketInterval);
  static int applicationPort = 11000;

  Ptr<Ipv4> remoteIpv4 = ueNode->GetObject<Ipv4>();
  Ipv4Address remoteIpAddr =
      remoteIpv4->GetAddress(1, 0).GetLocal(); // Interface 0 is loopback

  UdpServerHelper server(applicationPort);
  ApplicationContainer apps = server.Install(ueNode);
  apps.Start(Simulator::Now());
  apps.Stop(management_interval);

  uint32_t MaxPacketSize = 1024;

  UdpClientHelper client(remoteIpAddr, applicationPort);
  client.SetAttribute("Interval", TimeValue(udpInterval));
  client.SetAttribute("PacketSize", UintegerValue(MaxPacketSize));
  ApplicationContainer appc = client.Install(targetServer);
  appc.Start(Simulator::Now());
  appc.Stop(management_interval);

  ++applicationPort;
}

void request_video(Ptr<Node> sender_node, Ptr<Node> receiver_node,
                   Ipv4Address targetServerAddress)
{
  static uint16_t m_port = 2000;
  static int request_id = 0;

  EvalvidServerHelper server(m_port);
  server.SetAttribute("SenderTraceFilename", StringValue("st_highway_cif.st"));

  server.SetAttribute("SenderDumpFilename",
                      StringValue("sd_" + std::to_string(request_id) + ".txt"));
  // server.SetAttribute("PacketPayload", UintegerValue(512));

  ApplicationContainer apps = server.Install(sender_node);
  apps.Start(Seconds(0));

  EvalvidClientHelper client(targetServerAddress, m_port);
  client.SetAttribute("ReceiverDumpFilename",
                      StringValue("rd_" + std::to_string(request_id) + ".txt"));
  apps = client.Install(receiver_node);
  apps.Start(Seconds(0));

  request_id++;
  m_port++;
}

void UDPApp(Ptr<Node> remoteHost, NodeContainer ueNodes)
{
  // Install and start applications on UEs and remote host

  ApplicationContainer serverApps;
  ApplicationContainer clientApps;
  Time interPacketInterval = MilliSeconds(50);
  uint16_t dlPort = 1100;
  uint16_t ulPort = 2000;
  int startTime = 2;
  Ptr<Ipv4> remoteIpv4 = remoteHost->GetObject<Ipv4>();
  Ipv4Address remoteIpAddr =
      remoteIpv4->GetAddress(1, 0).GetLocal(); // Interface 0 is loopback

  for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
  {
    Ptr<Node> ue = ueNodes.Get(u);
    Ptr<Ipv4> ueIpv4 = ue->GetObject<Ipv4>();
    Ipv4Address ueIpAddr = ueIpv4->GetAddress(1, 0).GetLocal();
    ulPort++;

    if (!disableDl)
    {
      PacketSinkHelper dlPacketSinkHelper(
          "ns3::UdpSocketFactory",
          InetSocketAddress(Ipv4Address::GetAny(), dlPort));
      serverApps.Add(dlPacketSinkHelper.Install(ue));

      UdpClientHelper dlClient(ueIpAddr, dlPort);
      dlClient.SetAttribute("Interval", TimeValue(interPacketInterval));
      dlClient.SetAttribute("MaxPackets", UintegerValue(1000000));
      dlClient.SetAttribute("PacketSize", UintegerValue(1024));
      clientApps.Add(dlClient.Install(remoteHost));
    }

    if (!disableUl)
    {
      ++ulPort;
      PacketSinkHelper ulPacketSinkHelper(
          "ns3::UdpSocketFactory",
          InetSocketAddress(Ipv4Address::GetAny(), ulPort));
      serverApps.Add(ulPacketSinkHelper.Install(remoteHost));

      UdpClientHelper ulClient(remoteIpAddr, ulPort);
      ulClient.SetAttribute("Interval", TimeValue(interPacketInterval));
      ulClient.SetAttribute("MaxPackets", UintegerValue(1000000));
      ulClient.SetAttribute("PacketSize", UintegerValue(1024));
      clientApps.Add(ulClient.Install(ue));
    }
  }

  serverApps.Start(Seconds(1));
  clientApps.Start(Seconds(startTime));
}

void ThroughputMonitor(FlowMonitorHelper *fmhelper, Ptr<FlowMonitor> flowMon)
{
  // count lost packets
  flowMon->CheckForLostPackets();

  uint32_t LostPacketsum = 0;
  float PDR, PLR, Delay, Throughput;
  auto flowStats = flowMon->GetFlowStats();

  Ptr<Ipv4FlowClassifier> classing =
      DynamicCast<Ipv4FlowClassifier>(fmhelper->GetClassifier());
  std::ofstream qos_file;

  for (auto stats : flowStats)
  {

    // disconsider old flows by checking if first packet was sent before this
    // round management interval
    if (Simulator::Now() - stats.second.timeFirstTxPacket >
        management_interval)
    {
      continue;
    }

    // find flow characteristics
    Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow(stats.first);

    PDR = (double)(100 * stats.second.rxPackets) / (stats.second.txPackets);
    LostPacketsum = (double)(stats.second.txPackets) - (stats.second.rxPackets);
    PLR = (double)(LostPacketsum * 100) / stats.second.txPackets;
    Delay = (stats.second.delaySum.GetSeconds()) / (stats.second.txPackets);
    Throughput = stats.second.rxBytes * 8.0 /
                 (stats.second.timeLastRxPacket.GetSeconds() -
                  stats.second.timeFirstTxPacket.GetSeconds()) /
                 1024 / 1024;

    std::cout << "Flow ID     : " << stats.first << " ; "
              << fiveTuple.sourceAddress << " -----> "
              << fiveTuple.destinationAddress << std::endl;
    std::cout << "Tx Packets = " << stats.second.txPackets << std::endl;
    std::cout << "Rx Packets = " << stats.second.rxPackets << std::endl;
    std::cout << "Lost Packets = "
              << (stats.second.txPackets) - (stats.second.rxPackets)
              << std::endl;
    std::cout << "Packets Delivery Ratio (PDR) = " << PDR << "%" << std::endl;
    std::cout << "Packets Lost Ratio (PLR) = " << PLR << "%" << std::endl;
    std::cout << "Delay = " << Delay << " Seconds" << std::endl;
    std::cout << "Total Duration    : "
              << stats.second.timeLastRxPacket.GetSeconds() -
                     stats.second.timeFirstTxPacket.GetSeconds()
              << " Seconds" << std::endl;
    std::cout << "Last Received Packet  : "
              << stats.second.timeLastRxPacket.GetSeconds() << " Seconds"
              << std::endl;
    std::cout << "Throughput: " << Throughput << " Mbps" << std::endl;
    std::cout << "Throughput in bytes: " << Throughput * 12500000 << " Bps" << std::endl;
    // receiving node, used to catch user downlink traffic
    NS_LOG_DEBUG("target node = " << get_user_id_from_ipv4(
            fiveTuple.destinationAddress));
    std::cout << "-------------------------------------------------------------"
                 "--------------"
              << std::endl;

    // received id will be -1 in case it is not a mobile user
    int receiver_id = get_user_id_from_ipv4(fiveTuple.destinationAddress);
    if (receiver_id != -1)
    {
      user_throughput[receiver_id] = Throughput * 12500000; // save in bytes
      //
      cell_throughput[getCellId(receiver_id)] += Throughput;
    }
  }

  // schedule itself in 1sec
  Simulator::Schedule(management_interval, ThroughputMonitor, fmhelper,
                      flowMon);
}

void UAVManager()
{
  // get centers from python script
  exec("python3 scratch/clustering.py");
  std::ifstream centroids("centroids.txt");
  std::vector<std::pair<int, int>> centers;
  double tmp_x, tmp_y;
  while (centroids >> tmp_x >> tmp_y)
  {
    centers.push_back({tmp_x, tmp_y});
    // NS_LOG_DEBUG("tmp_x " << tmp_x << " tmp_y " << tmp_y);
  }

  for (uint32_t i = 0; i < numUAVs; i++)
  {
    Ptr<Node> drone = uavNodes.Get(i);
    int closest_hot_spot_index = get_closest_center_index(drone, centers);
    if (closest_hot_spot_index == -1)
    {
      continue;
    }

    if (algorithm == "iuavbs")
    {
      std::pair<int, int> closest_hot_spot = centers[closest_hot_spot_index];
      if (drones_in_use[i] == false and
          hot_spots_served[closest_hot_spot_index] == false)
      {
        move_uav(drone,
                    Vector(closest_hot_spot.first, closest_hot_spot.second, 20),
                    100);
        drones_in_use[i] = true;
        hot_spots_served[closest_hot_spot_index] = true;
      }
    }
  }

  // zero neighbors matrix after repositioning drones
  if (algorithm == "iavbs")
  {
    for (uint32_t i = 0; i < numBSs; i++)
      for (uint32_t u = 0; u < numUEs; u++)
        neighbors[i][u] = 0;
  }

  centroids.close();

  Simulator::Schedule(management_interval, &UAVManager);
  return;
}

bool find_handover(Handover h)
{
  for (auto &handover_compare : handover_vector)
  {
    if (h == handover_compare)
    {
      // NS_LOG_DEBUG("Handover already requested, not repeating.");
      return true;
    }
  }
  return false;
}

void schedule_handover(int id_user, int id_source, int id_target)
{
  // inter drones handovers may be tricky
  bool block_drones_handovers = true;
  bool random_time = false;
  int h_time = (int)Simulator::Now().GetSeconds();
  int max_handovers_per_second = 3;

  // reject handover if in this second more than the limit have been performed
  if (handovers_per_second.find(h_time) != handovers_per_second.end())
  {
    if (handovers_per_second[h_time] > max_handovers_per_second)
      return;
  }

  if (block_drones_handovers)
  {
    if (is_drone(id_target) && is_drone(id_source))
      return;
  }

  // create handover identifier
  Handover handover(Simulator::Now().GetSeconds(), id_user, id_source,
                    id_target);

  // if this handover has already been attempted, return.
  if (find_handover(handover))
  {
    // NS_LOG_DEBUG("Handover already exists");
    return;
  }

  // reset x2 interface before handover
  // I dont know anymore what to do to make this work
  id_source = get_cell(id_user);

  Ptr<LteUeNetDevice> ueLteDevice =
      ueDevs.Get(id_user)->GetObject<LteUeNetDevice>();
  Ptr<LteUeRrc> ueRrc = ueLteDevice->GetRrc();

  // NS_LOG_DEBUG("User device in state " << ueRrc->GetState());
  if (ueRrc->GetState() != LteUeRrc::CONNECTED_NORMALLY)
  {
    // NS_LOG_DEBUG("Wrong LteUeRrc state!");
    return;
  }
  // NS_TEST_ASSERT_MSG_EQ (ueRrc->GetState (), LteUeRrc::CONNECTED_NORMALLY,
  // "Wrong LteUeRrc state!");

  Ptr<NetDevice> enbDev = enbDevs.Get(id_source);
  if (enbDev == 0)
  {
    NS_LOG_WARN(Simulator::Now().GetSeconds()
			<< " LTE eNB " << id_source << " device not found");
    return;
  }

  Ptr<LteEnbNetDevice> enbLteDevice = enbDev->GetObject<LteEnbNetDevice>();
  Ptr<LteEnbRrc> enbRrc = enbLteDevice->GetRrc();
  // uint16_t rnti = ueRrc->GetRnti();

  // NS_LOG_DEBUG("rnti " << rnti);
  // NS_LOG_DEBUG("user_id " << id_user);
  // NS_LOG_DEBUG("id_source " << id_source);
  // NS_LOG_DEBUG("id_target " << id_user);

  // this mf keeps returning error I dont know why
  // Ptr<UeManager> ueManager = enbRrc->GetUeManager(rnti);

  if (random_time)
  {
    // generate random time for the handover in order not to overcrowd random access
    std::uniform_real_distribution<> dis(0, 1.0);
    int handover_time = dis(generator);

    handover = Handover(handover_time, id_user, id_source, id_target);
    NS_LOG_INFO(handover);
    handover_vector.push_back(handover);
    lteHelper->HandoverRequest(Seconds(handover_time), ueDevs.Get(id_user),
                               enbDevs.Get(id_source), enbDevs.Get(id_target));
  }
  else
  {
    handover = Handover(Simulator::Now().GetSeconds(), id_user, id_source, id_target);
    NS_LOG_INFO(handover);
    handover_vector.push_back(handover);
    lteHelper->HandoverRequest(Simulator::Now(), ueDevs.Get(id_user),
                               enbDevs.Get(id_source), enbDevs.Get(id_target));
  }

  // control number of handovers performee
  if (handovers_per_second.find(h_time) == handovers_per_second.end())
    handovers_per_second[h_time] = 1;
  else
    handovers_per_second[h_time] += 1;
  // if handover is valid, add it to list of handovers
}

void handoverManager(std::string path)
{

  int nodeid = get_nodeid_from_path(path);

  if (nodeid == -1)
    return;

  int user_thr = 5 * 1024 * 1024;
  int cell_thr = 10 * 1024 * 1024;

  // user-wise strongest cell implementation
  int imsi = nodeid + 1;
  uint32_t servingCell = get_cell(nodeid);
  double rsrp = -9999999;
  uint32_t bestNeighborCell = -1;
  int signal_threshold = 3;

  if ((int)servingCell == -1)
  {
    return;
  }

  if (handover_policy == "iuavbs")
  {
    // if user is not served
    if (unserved_users[nodeid])
    {
      // handover to closest drone
      for (uint32_t cell = 0; cell < numBSs; cell++)
      {
        // define drone with highest signal as bset cell
        if (neighbors[cell][nodeid] > rsrp && cell != servingCell && is_drone(cell))
        {
          rsrp = neighbors[cell][nodeid];
          bestNeighborCell = cell;
        }
        if ((int)bestNeighborCell != -1 && bestNeighborCell != servingCell && Simulator::Now() > Seconds(1))
        {

          if (enableHandover)
          {
            schedule_handover(nodeid, servingCell, bestNeighborCell);
          }
        }
      }
    }
  }

  else if (handover_policy == "competing")
  {
    if (user_throughput[nodeid] >= user_thr)
    {
      for (uint32_t cell = 0; cell < numBSs; cell++)
      {
        if (neighbors[cell][nodeid] > rsrp && cell != servingCell &&
            cell_throughput[cell] < cell_thr)
        {
          rsrp = neighbors[cell][nodeid];
          bestNeighborCell = cell;
        }
      }
    }

    if (rsrp > (signal_threshold + neighbors[servingCell][imsi - 1]) &&
        neighbors[servingCell][imsi - 1] != 0 &&
        bestNeighborCell != servingCell)
    {

      if (enableHandover)
      {
        schedule_handover(nodeid, servingCell, bestNeighborCell);
      }
    }
  }

  // this is taking a lot of time, why?
  else if (handover_policy == "classic")
  {
    for (uint32_t cell = 0; cell < numBSs; cell++)
    {
      if (neighbors[cell][nodeid] > rsrp && cell != servingCell)
      {
        rsrp = neighbors[cell][nodeid];
        bestNeighborCell = cell;
      }
    }

    if (rsrp > (signal_threshold + neighbors[servingCell][imsi - 1]) &&
        neighbors[servingCell][imsi - 1] != 0 &&
        bestNeighborCell != servingCell)
    {
      if (enableHandover)
      {
        schedule_handover(nodeid, servingCell, bestNeighborCell);
      }
    }
  }
  else if (handover_policy == "none")
    return;
  else
  {
    NS_FATAL_ERROR("Handover policy type invalid.");
  }
}

// function that receives the user node container and checks if requirements are being met
double check_served(NodeContainer ueNodes)
{
    int counter_served = 0;

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        for (uint32_t u = 0; u < numStaticCells; ++u)
        {
            if (cellUe[u][i])
            {
                counter_served++;
                continue;
            }
        }
    }

    return (double)counter_served / ueNodes.GetN();
}

// function that receives the user node container and returns the percentage covered
double check_coverage(NodeContainer ueNodes)
{
    int counter_covered = 0;

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        for (uint32_t u = 0; u < BSNodes.GetN(); ++u)
        {
            if (edgeUe[u][i])
            {
                counter_covered++;
                continue;
            }
        }
    }

    return (double)counter_covered / ueNodes.GetN();
}

// migrations manager
void manager()
{
    double weights[3] = {57, 14, 28};

    Simulator::Schedule(managerInterval, &manager);

    std::cout << "manager started at " << Simulator::Now().GetSeconds() << " \n";

    // a counter to see the percentage of users who
    //  are served with the minimum application requirements
    float served_with_reqs = 0;
    NS_LOG_UNCOND("Percentage of users covered: " << check_coverage(ueNodes));
    NS_LOG_UNCOND("Percentage of users served: " << check_served(ueNodes));

    for (uint32_t i = 0; i < BSNodes.GetN(); ++i)
    {
        std::cout << "server n " << i << " with " << resources[i] << " resource units\n";
    }

    std::cout << "..................................\n\n\n";

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        // check if node is being served

		int result;
        unsigned int serving_node;

		result = getEdge(i);

        if (result >= 0)
        {
			serving_node = static_cast<unsigned int>(result);
			NS_LOG_UNCOND("Serving node: " << serving_node);
            if (serving_node < numStaticCells)
            {
                NS_LOG_UNCOND("Node " << i << " being served by edge");
                latency.push_back(serverReqs[0][0]);
                cost.push_back(serverReqs[0][1]);
            }
            else if (serving_node < numStaticCells + numUAVs)
            {
                NS_LOG_UNCOND("Node " << i << " being served by uav edge");
                latency.push_back(serverReqs[1][0]);
                cost.push_back(serverReqs[1][1]);
            }
            else
            {
                NS_LOG_UNCOND("Node " << i << " being served by cloud");
                latency.push_back(serverReqs[3][0]);
                cost.push_back(serverReqs[3][1]);
            }

            if (latency.back() < applicationReqs[applicationType][0])
            {
                served_with_reqs++;
            }

            if (algorithm == "nomigration" || algorithm == "greedy")
                continue;

            int bestEdgeServer = -1;
            int greatestScore = -1;
            unsigned int edgeId = 0;

            // I will just assume that the predictions are right
            HandoverPrediction(i, 5);

            // if a handover is going to happen
            if (Seconds(handoverPredictions[i][0]) > Simulator::Now())
            {
                while (edgeId < BSNodes.GetN())
                {
                    double score = 0;

                    // server characteristics
                    double latency_score = 0;
                    double cost_score = 0;
                    double resource_score = 0;
                    // if I cared a bit more I'd use the limits.h
                    //  library to set this to the max, but I don't
                    int server_latency = 999999;

                    // get server metrics
                    if (edgeId < numStaticCells)
                    {
                        latency_score += (1 / serverReqs[0][0]) * weights[0];
                        resource_score += serverReqs[0][1] * weights[1];
                        cost_score += (1 / serverReqs[0][2]) * weights[2];
                        server_latency = serverReqs[0][0];
                    }
                    else if (edgeId < numStaticCells + numUAVs)
                    {
                        latency_score = (1 / serverReqs[1][0]) + weights[0];
                        resource_score = (serverReqs[1][1]) + weights[1];
                        cost_score = (1 / serverReqs[1][2]) + weights[2];
                        server_latency = serverReqs[1][0];
                    }
                    else
                    {
                        latency_score = (1 / serverReqs[3][0]) * weights[0];
                        resource_score = (serverReqs[3][1]) * weights[1];
                        cost_score = (1 / serverReqs[3][2]) * weights[2];
                        server_latency = serverReqs[3][0];
                    }

                    // Get the last insertion to the latency container
                    // and check if requirements are being met
                    // or if server is out of resources
                    if (server_latency > applicationReqs[applicationType][0] ||
                        resources[edgeId] == 0)
                        score = 0;
                    else
                        score = latency_score + cost_score + resource_score;

                    // in the case of qos-based
                    if (algorithm == "qos")
                        score = 1 / serverReqs[1][0];

                    NS_LOG_UNCOND(Simulator::Now().GetSeconds() << " -- server " << edgeId << " score: " << score);

                    // get greated score
                    if (score > greatestScore)
                    {
                        greatestScore = score;
                        bestEdgeServer = edgeId;
                    }
                    edgeId++;
                }
                if (bestEdgeServer != (int)serving_node)
                {
                    if (edgeMigrationChart[i][bestEdgeServer] + 5 > Simulator::Now().GetSeconds())
                        ; // do nothing
                    // return;
                    else
                    {
                        migrate(BSNodes.Get(serving_node), BSNodes.Get(bestEdgeServer),
                                x2AddressMatrix[serving_node][bestEdgeServer], x2AddressMatrix[bestEdgeServer][serving_node]);
                        edgeMigrationChart[i][bestEdgeServer] = Simulator::Now().GetSeconds();
                        edgeUe[serving_node][i] = 0;
                        edgeUe[bestEdgeServer][i] = 1;
                    }
                }
            }

            // renew applications periodically
            requestApplication(ueNodes.Get(i), BSNodes.Get(serving_node));
        }
        else
        {
            NS_LOG_UNCOND("Node " << i << " not being served?");
        }
    }
    // NS_LOG_UNCOND("Users served with the minimun reqs: " << served_with_reqs / ueNodes.GetN());
}

double vec_average(std::vector<double> vec)
{

  int i = 0;
  double sum = 0;
  for (auto &v : vec)
  {
    i++;
    sum += v;
  }
  return sum / i;
}

void print_connections()
{
  for (uint32_t u = 0; u < numUEs; u++)
  {
    NS_LOG_DEBUG(getCellId(u));
  }

  NS_LOG_DEBUG("");

  for (uint32_t i = 0; i < numBSs; i++)
  {
    for (uint32_t u = 0; u < numUEs; u++)
    {
      std::cout << cellUe[i][u] << " ";
    }
    NS_LOG_DEBUG("");
  }
}

void just_a_monitor()
{
  // percentage t f throughput below requested to consider the user as
  // underved
  double lenience = 0.8; // unserved if below 80% of requested

  // increase monitor calls counter
  monitor_calls++;

  // open blank file for positions of unserved users
  std::ofstream unserved("unserved.txt", std::ofstream::out);

  // tmp variables
  double number_of_unserved = 0;
  double cell_thr_sum = 0;
  double uav_usage_total = 0;
  double user_throughput_total = 0;

  // print_connections();

  // check if more than 1sec has passed, as before this users may not be
  // connected
  if (Simulator::Now() > Seconds(1))
  {
    // loop over all users
    for (uint32_t i = 0; i < numUEs; i++)
    {

      double throughput = user_throughput[i];
      user_throughput_total += user_throughput[i];

      NS_LOG_DEBUG("User " << i << " throughput " << throughput);
      NS_LOG_DEBUG("User " << i << " request value " << user_requests[i]);

      // user marked as unserved;
      if (lenience * user_requests[i] < throughput)
      {
        Vector user_pos =
            ueNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        unserved << i << " " << user_pos.x << " " << user_pos.y << " "
                 << user_requests[i] << "\n";
        number_of_unserved++;
        unserved_users[i] = true;
      }

      int cell = getCellId(i);
      if (cell != -1)
      {
        std::string cell_type = is_drone(cell) ? "UAV" : "GBS";
        NS_LOG_DEBUG("User " << i << " is in cell " << cell << " " << cell_type << ".");

        if (cell_type == "UAV")
        {
          uav_usage_total++;
        }
      }
      else
      {
        NS_LOG_DEBUG("User is not connected.");
      }
    }
  }

  for (uint32_t i = 0; i < numBSs; i++)
  {
    double cell_thr = 0; // cell throughput in Mbps
    for (uint32_t u = 0; u < numUEs; u++)
    {
      if (cellUe[i][u])
      {
        cell_thr += user_throughput[u]; // not very precise, does not consider
        // time window...
        cell_thr_sum += cell_thr;
      }
    }
    NS_LOG_DEBUG(Simulator::Now().GetSeconds()
        << "s Cell " << i << " usage: " << cell_thr);
  }

  // update metrics peek
  peek_service_level.push_back(1 - (number_of_unserved / numUEs));
  peek_cell_throughput.push_back(cell_thr_sum);
  peek_uav_usage.push_back(uav_usage_total / numBSs);
  peek_user_throughput.push_back(user_throughput_total / numUEs);

  // print metrics peek
  NS_LOG_DEBUG("Simulation time: " << Simulator::Now().GetSeconds());
  NS_LOG_DEBUG("peek_service_level " << peek_service_level.back());
  NS_LOG_DEBUG("peek_uav_usage " << peek_uav_usage.back());
  NS_LOG_DEBUG("peek_cell_throughput " << peek_cell_throughput.back());
  NS_LOG_DEBUG("peek_user_throughput " << peek_user_throughput.back());

  unserved.close();
  Simulator::Schedule(management_interval, &just_a_monitor);
}

int main(int argc, char* argv[])
{
	uint32_t seed = 4242;
	short simTime = 100;

	LogComponentEnable ("uav-edge", LOG_LEVEL_INFO);
	LogComponentEnable ("EvalvidClient", LOG_LEVEL_INFO);
	LogComponentEnable ("EvalvidServer", LOG_LEVEL_INFO);
	//LogComponentEnable ("Ipv4StaticRouting", ns3::LOG_LEVEL_ALL);

	CommandLine cmd;
	
	cmd.AddValue("numSCs", "how many static cells are in the simulation", numStaticCells);
	cmd.AddValue("numUAVs", "how many UAVs are in the simulation", numUAVs);
	cmd.AddValue("numUEs", "how many UEs are in the simulation", numUEs);
	cmd.AddValue("seed", "random seed value.", seed);
	cmd.Parse(argc, argv);

	ns3::RngSeedManager::SetSeed(seed);
	ns3_dir = GetTopLevelSourceDir();

	// make sure there's hot spots even if the number of uavs is 0
	number_of_hot_spots = numUAVs == 0 ? 10 : numUAVs;
	numBSs = numStaticCells + numUAVs;
	numEdgeServers = numBSs;
	initialize_vectors();

	Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();

	Ptr<Node> pgw = config_LTE(lteHelper, epcHelper);

	NodeContainer remoteHostContainer;
	remoteHostContainer.Create(1);
	Ptr<Node> remoteHost = remoteHostContainer.Get(0);

	// create nodes in global containers
	staticBSNodes.Create(numStaticCells);
	uavNodes.Create(numUAVs);
	ueNodes.Create(numUEs);

	// merge uav and bs nodes
	BSNodes = NodeContainer(staticBSNodes, uavNodes);

	InternetStackHelper internet;
	internet.Install(remoteHostContainer);
    internet.Install(ueNodes);

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
	for (uint32_t u = 0; u < ueNodes.GetN(); ++u) {
		Ptr<Node> ueNode = ueNodes.Get(u);
		Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
		ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
	}

	Ptr<Node> sgw = epcHelper->GetSgwNode();
	Ptr<Ipv4> sgwIpv4 = sgw->GetObject<Ipv4>();
	Ptr<Ipv4StaticRouting> sgwStaticRouting = ipv4RoutingHelper.GetStaticRouting(sgwIpv4);
	uint32_t sgw2pgwInterface = sgwIpv4->GetInterfaceForPrefix(Ipv4Address("14.0.0.0"),Ipv4Mask("255.255.255.0"));
	sgwStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.255.255.0"), sgw2pgwInterface);

	NodeContainer staticNodes;
	staticNodes.Add(epcHelper->GetSgwNode());
	staticNodes.Add(pgw);
	staticNodes.Add(remoteHost);
	install_mobility(staticNodes, staticBSNodes, uavNodes, ueNodes);

	install_LTE(lteHelper, epcHelper, staticBSNodes, uavNodes, ueNodes);

	for (uint32_t u = 0; u < staticBSNodes.GetN(); ++u) {
		Ptr<Node> BSNode = staticBSNodes.Get(u);
		Ptr<Ipv4> BSIpv4 = BSNode->GetObject<Ipv4>();
		Ptr<Ipv4StaticRouting> BSStaticRouting = ipv4RoutingHelper.GetStaticRouting(BSIpv4);
		BSStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.255.255.0"), 1);
	}

	for (uint32_t u = 0; u < uavNodes.GetN(); ++u) {
		Ptr<Node> uavNode = uavNodes.Get(u);
		Ptr<Ipv4> uavIpv4 = uavNode->GetObject<Ipv4>();
		Ptr<Ipv4StaticRouting> uavStaticRouting = ipv4RoutingHelper.GetStaticRouting(uavIpv4);
		uavStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.255.255.0"), 1);
	}

	installEnergy (uavNodes);

	// populate user ip map
	for (uint32_t i = 0; i < ueNodes.GetN(); i++)
	{
		Ptr<Ipv4> remoteIpv4 = ueNodes.Get(i)->GetObject<Ipv4>();
		Ipv4Address remoteIpAddr = remoteIpv4->GetAddress(1, 0).GetLocal();
		user_ip[i] = remoteIpAddr;
	}

	AnimationInterface animator("lte.xml");
	animator.SetMobilityPollInterval(Seconds(1));
    for (uint32_t i = 0; i < staticBSNodes.GetN(); ++i) {
		animator.UpdateNodeDescription(staticBSNodes.Get(i), "SC " + std::to_string(i));
		animator.UpdateNodeColor(staticBSNodes.Get(i), 0, 200, 45);
		animator.UpdateNodeSize(staticBSNodes.Get(i)->GetId(),10,10); // to change the node size in the animation.
    }
    for (uint32_t i = 0; i < uavNodes.GetN(); ++i) {
		animator.UpdateNodeDescription(uavNodes.Get(i), "UAV " + std::to_string(i));
		animator.UpdateNodeColor(uavNodes.Get(i), 250, 200, 45);
		animator.UpdateNodeSize(uavNodes.Get(i)->GetId(),10,10); // to change the node size in the animation.
    }
    for (uint32_t j = 0; j < ueNodes.GetN(); ++j) {
		animator.UpdateNodeDescription(ueNodes.Get(j), "UE " + std::to_string(j+1));
		animator.UpdateNodeColor(ueNodes.Get(j), 20, 10, 145);
		animator.UpdateNodeSize(ueNodes.Get(j)->GetId(),10,10);
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
	flowMonitor = flowHelper.Install(BSNodes);
	flowMonitor = flowHelper.Install(ueNodes);
	Simulator::Schedule(Seconds(simTime-0.001), NetworkStatsMonitor, &flowHelper, flowMonitor);

	// populate centroids
	auto centers = create_hot_spots();

	// populate simulation with requests trace
	auto requests = populate_requests_trace();

	Simulator::Schedule(Seconds(MOBILITY_ENERGY_INTERVAL), &update_mobility_energy, uavNodes);
	Simulator::Schedule(Seconds(COMMS_ENERGY_INTERVAL), &update_comms_energy, uavNodes);
	Simulator::Schedule(Seconds(COMPUTE_ENERGY_INTERVAL), &initial_compute_energy, uavNodes);
	Simulator::Schedule(management_interval, ThroughputMonitor, &flowHelper, flowMonitor); // recurrent
  Simulator::Schedule(management_interval, &UAVManager);                           // only executed in the beginning?
  Simulator::Schedule(management_interval, &just_a_monitor);                 // just a monitor
  Simulator::Schedule(Simulator::Now(), &manager);

	/* handover reporting callbacks*/
  Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                  MakeCallback(&NotifyHandoverStartEnb));
  Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
                  MakeCallback(&NotifyConnectionEstablishedUe));
  Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
                  MakeCallback(&NotifyHandoverStartUe));
  Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                  MakeCallback(&NotifyHandoverEndOkEnb));

  Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
                  MakeCallback(&NotifyHandoverEndOkUe));

  /* signal reporting callbacks */
  Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/LteUePhy/"
                  "ReportUeMeasurements",
                  MakeCallback(&ReportUeMeasurementsCallback));
  Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/RecvMeasurementReport",
                  MakeCallback(&RecvMeasurementReportCallback));
  // Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/PhySyncDetection",
  //                 MakeCallback(&PhySyncDetectionCallback));
  // Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/RadioLinkFailure",
  //                 MakeCallback(&RadioLinkFailureCallback));

  lteHelper->EnableTraces(); // enable all traces

	Simulator::Stop(Seconds(simTime));
	Simulator::Run();

	Simulator::Destroy();
	return 0;
}
