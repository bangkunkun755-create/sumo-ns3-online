// Live nearest-RSU application destination switching, NOT L2/cellular handover.
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <vector>
#include <limits>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace ns3;

namespace
{
// This is a host TCP socket to Python, separate from the simulated UDP sockets.
class Bridge
{
  public:
    explicit Bridge(uint16_t port)
    {
        m_fd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (m_fd < 0)
        {
            throw std::runtime_error("Cannot create host TCP socket");
        }
        timeval timeout{30, 0};
        ::setsockopt(m_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        ::setsockopt(m_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(port);
        ::inet_pton(AF_INET, "127.0.0.1", &address.sin_addr);
        if (::connect(m_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0)
        {
            ::close(m_fd);
            m_fd = -1;
            throw std::runtime_error("Cannot connect to Python bridge");
        }
    }

    ~Bridge()
    {
        if (m_fd >= 0)
        {
            ::close(m_fd);
        }
    }

    std::string Exchange(const std::string& request)
    {
        size_t sent = 0;
        while (sent < request.size())
        {
            const auto count = ::send(m_fd, request.data() + sent, request.size() - sent, MSG_NOSIGNAL);
            if (count <= 0)
            {
                if (count < 0 && errno == EINTR)
                {
                    continue;
                }
                throw std::runtime_error("Bridge send failed or timed out");
            }
            sent += static_cast<size_t>(count);
        }
        std::string response;
        char ch;
        while (response.size() < 4096)
        {
            const auto count = ::recv(m_fd, &ch, 1, 0);
            if (count <= 0)
            {
                if (count < 0 && errno == EINTR)
                {
                    continue;
                }
                throw std::runtime_error("Bridge disconnected or receive timed out");
            }
            if (ch == '\n')
            {
                return response;
            }
            response.push_back(ch);
        }
        throw std::runtime_error("Bridge response exceeds frame limit");
    }

  private:
    int m_fd{-1};
};

struct PacketRecord
{
    uint32_t rsu;
    double sendTime;
    double distance;
    bool accepted;
    bool received{false};
    double arriveTime{0};
    double delayMs{0};
};

std::vector<Vector> ReadStations(const std::string& path)
{
    std::ifstream input(path);
    std::string line;
    if (!std::getline(input, line) || line.find("rsu_id,x_m,y_m,z_m") != 0)
        throw std::runtime_error("Invalid RSU layout header");
    std::vector<Vector> locations;
    while (std::getline(input, line))
    {
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream row(line);
        uint32_t id;
        double x, y, z;
        if (!(row >> id >> x >> y >> z) || id != locations.size() + 1
            || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            throw std::runtime_error("Invalid RSU layout row");
        locations.emplace_back(x, y, z);
    }
    if (locations.size() < 2 || locations.size() > 32)
        throw std::runtime_error("RSU count must be 2..32");
    return locations;
}

struct Experiment
{
    Bridge* bridge{};
    NodeContainer nodes;
    std::ofstream positions;
    std::ofstream packets;
    std::map<uint32_t, PacketRecord> records;
    std::vector<uint32_t> generated, accepted, received, previousReceived;
    std::vector<double> totalDelay;
    std::vector<Ptr<Socket>> senders;
    std::vector<Vector> stations;
    std::ofstream association;
    uint32_t serving{0}, changes{0};
    uint32_t duplicates{0};
    uint32_t malformed{0};
    uint32_t syncCount{0};
    double maxPositionError{0};
    double maxClockError{0};
    double maxPredictionCorrection{0};

    double durationSeconds{100};
    std::string progress{"summary"};

    void PrintProgress(double now)
    {
        if (progress == "off")
        {
            return;
        }
        std::ostringstream line;
        line << std::fixed << std::setprecision(1)
             << "[PROGRESS] t=" << now << "/" << durationSeconds << "s target=RSU" << serving;
        const auto car = nodes.Get(0)->GetObject<MobilityModel>()->GetPosition();
        uint32_t totalTx = 0, totalRx = 0, newlyReceived = 0;
        for (uint32_t i = 0; i < stations.size(); ++i)
        {
            totalTx += generated[i];
            totalRx += received[i];
            newlyReceived += received[i] - previousReceived[i];
            previousReceived[i] = received[i];
        }
        line << " | TX=" << totalTx << " RX=" << totalRx << " (+" << newlyReceived << ")"
             << " distance=" << CalculateDistance(car, stations[serving - 1]) << "m"
             << " | target_TX=" << generated[serving - 1] << " target_RX=" << received[serving - 1];
        // Flush at each barrier so the Python launcher can display this during the run.
        std::cout << line.str() << std::endl;
    }

    void Sync(uint32_t seq)
    {
        const double now = Simulator::Now().GetSeconds();
        // Publish complete real packet events before Python displays this barrier.
        // This is host-side I/O only, not an ns-3 event or timing change.
        packets.flush();
        if (!packets)
        {
            throw std::runtime_error("Cannot flush live packet events");
        }
        std::ostringstream request;
        request << std::setprecision(17) << "STEP " << seq << " " << now;
        for (uint32_t i = 0; i < stations.size(); ++i)
            request << " " << generated[i] << " " << received[i];
        request << "\n";
        const auto response = bridge->Exchange(request.str());
        std::istringstream row(response);
        std::string kind;
        std::string extra;
        uint32_t ack;
        uint32_t vehicleCount, selected;
        double elapsed, sumoTime, offset, x, y, z, speed, vx, vy;
        if (!(row >> kind >> ack >> elapsed >> sumoTime >> offset >> x >> y >> z
                  >> speed >> vx >> vy >> vehicleCount >> selected) || (row >> extra)
            || kind != "STATE" || ack != seq || vehicleCount != 1)
        {
            throw std::runtime_error("Invalid bridge acknowledgement: " + response);
        }
        for (double value : {elapsed, sumoTime, offset, x, y, z, speed, vx, vy})
        {
            if (!std::isfinite(value))
            {
                throw std::runtime_error("Non-finite live state");
            }
        }
        const double clockError = std::max(std::abs(now - elapsed),
                                          std::abs(sumoTime - offset - now));
        if (clockError > 1e-8)
        {
            throw std::runtime_error("SUMO and ns-3 simulation clocks differ");
        }
        auto mobility = nodes.Get(0)->GetObject<ConstantVelocityMobilityModel>();
        const Vector previousPrediction = mobility->GetPosition();
        const Vector truth(x, y, z);
        const double correction = seq == 0 ? 0 : CalculateDistance(previousPrediction, truth);
        mobility->SetPosition(truth);
        mobility->SetVelocity(Vector(vx, vy, 0));
        const auto actual = mobility->GetPosition();
        uint32_t nearest = 1;
        double nearestDistance = std::numeric_limits<double>::infinity();
        for (uint32_t i = 0; i < stations.size(); ++i)
        {
            double distance = CalculateDistance(actual, stations[i]);
            if (distance < nearestDistance - 1e-9)
            {
                nearestDistance = distance;
                nearest = i + 1;
            }
        }
        if (selected != nearest)
            throw std::runtime_error("Python target differs from independently computed nearest RSU");
        const uint32_t previous = serving;
        serving = selected;
        if (previous && previous != serving) ++changes;
        association << seq << "," << now << "," << previous << "," << serving << ","
                    << nearestDistance << "," << (previous != serving) << "\n";
        association.flush();
        if (previous != serving && progress != "off")
            std::cout << (previous ? "[HANDOVER] t=" : "[INITIAL_SELECTION] t=")
                      << now << "s RSU" << previous << " -> RSU" << serving
                      << " distance=" << nearestDistance << "m (application target only)" << std::endl;
        const double positionError = CalculateDistance(actual, truth);
        maxPositionError = std::max(maxPositionError, positionError);
        maxClockError = std::max(maxClockError, clockError);
        maxPredictionCorrection = std::max(maxPredictionCorrection, correction);
        positions << seq << "," << now << "," << sumoTime << "," << offset << ","
                  << x << "," << y << "," << actual.x << "," << actual.y << "," << speed
                  << "," << positionError << "," << clockError << "," << correction << ","
                  << CalculateDistance(actual, nodes.Get(1)->GetObject<MobilityModel>()->GetPosition());
        for (uint32_t i = 1; i < stations.size(); ++i)
            positions << "," << CalculateDistance(actual, stations[i]);
        for (uint32_t i = 0; i < stations.size(); ++i)
            positions << "," << generated[i] << "," << received[i];
        positions << "," << serving << "\n";
        ++syncCount;
        PrintProgress(now);
    }

    void SendNearest(uint32_t sequence)
    {
        if (!serving || serving > stations.size())
            throw std::runtime_error("No valid serving RSU");
        Send(senders[serving - 1], sequence, serving - 1);
    }

    void Send(Ptr<Socket> socket, uint32_t sequence, uint32_t rsu)
    {
        SeqTsHeader header;
        header.SetSeq(sequence);
        Ptr<Packet> packet = Create<Packet>(256 - header.GetSerializedSize());
        packet->AddHeader(header);
        ++generated[rsu];
        const bool ok = socket->Send(packet) == 256;
        accepted[rsu] += ok;
        const double now = Simulator::Now().GetSeconds();
        const auto car = nodes.Get(0)->GetObject<MobilityModel>()->GetPosition();
        const auto station = nodes.Get(rsu + 1)->GetObject<MobilityModel>()->GetPosition();
        const double distance = CalculateDistance(car, station);
        records.emplace(sequence, PacketRecord{rsu, now, distance, ok});
        packets << "{\"event\":\"tx\",\"seq\":" << sequence << ",\"rsu_id\":" << rsu + 1
                << ",\"send_time\":" << now << ",\"arrive_time\":null,\"delay_ms\":null"
                << ",\"distance_m\":" << distance << ",\"bytes\":256,\"socket_accepted\":"
                << (ok ? "true" : "false") << "}\n";
    }

    void Receive(uint32_t rsu, Ptr<Socket> socket)
    {
        while (Ptr<Packet> packet = socket->Recv())
        {
            SeqTsHeader header;
            if (packet->GetSize() != 256)
            {
                ++malformed;
                continue;
            }
            packet->RemoveHeader(header);
            auto found = records.find(header.GetSeq());
            if (found == records.end() || found->second.rsu != rsu
                || std::abs(found->second.sendTime - header.GetTs().GetSeconds()) > 1e-8)
            {
                ++malformed;
                continue;
            }
            auto& record = found->second;
            const double now = Simulator::Now().GetSeconds();
            const double delay = (Simulator::Now() - header.GetTs()).GetSeconds() * 1000;
            const bool unique = !record.received;
            if (unique)
            {
                record.received = true;
                record.arriveTime = now;
                record.delayMs = delay;
                ++received[rsu];
                totalDelay[rsu] += delay;
            }
            else
            {
                ++duplicates;
            }
            packets << "{\"event\":\"rx\",\"seq\":" << header.GetSeq()
                    << ",\"rsu_id\":" << rsu + 1 << ",\"send_time\":" << record.sendTime
                    << ",\"arrive_time\":" << now << ",\"delay_ms\":" << delay
                    << ",\"bytes\":256,\"unique\":" << (unique ? "true" : "false") << "}\n";
            if (progress != "off" && (progress == "packets" || (unique && received[rsu] == 1)))
            {
                std::ostringstream line;
                line << (progress == "packets" ? "[RX]" : "[FIRST_RX]")
                     << " RSU" << rsu + 1 << " seq=" << header.GetSeq()
                     << std::fixed << std::setprecision(9)
                     << " send_time=" << record.sendTime << " arrive_time=" << now
                     << std::setprecision(6) << " delay_ms=" << delay
                     << " cumulative_received=" << received[rsu]
                     << " unique=" << (unique ? "true" : "false");
                std::cout << line.str() << std::endl;
            }
        }
    }

    void WriteResults(const std::filesystem::path& output, double duration,
                      double interval, double power, uint32_t seed, uint64_t run)
    {
        std::ofstream summary(output / "packet_summary.csv");
        summary << std::setprecision(17)
                << "seq,rsu_id,send_time,arrive_time,delay_ms,distance_m,socket_accepted,received\n";
        for (const auto& [seq, record] : records)
        {
            summary << seq << "," << record.rsu + 1 << "," << record.sendTime << ",";
            if (record.received)
            {
                summary << record.arriveTime;
            }
            summary << ",";
            if (record.received)
            {
                summary << record.delayMs;
            }
            summary << "," << record.distance << "," << record.accepted << "," << record.received << "\n";
        }
        std::ofstream results(output / "results.json");
        results << std::setprecision(17)
                << "{\n\"duration_s\":" << duration << ",\"sync_step_s\":1,\"sync_updates\":" << syncCount
                << ",\"max_position_error_m\":" << maxPositionError
                << ",\"max_clock_error_s\":" << maxClockError
                << ",\"max_prediction_correction_m\":" << maxPredictionCorrection
                << ",\"duplicates\":" << duplicates << ",\"malformed\":" << malformed
                << ",\"tx_power_dbm\":" << power << ",\"interval_s\":" << interval
                << ",\"rng_seed\":" << seed << ",\"rng_run\":" << run
                << ",\"arp_dead_timeout_s\":1,\"association_mode\":\"nearest\",\"selection_step_s\":1,\"handover_count\":" << changes
                << ",\"radio\":{\"standard\":\"802.11g\",\"mac\":\"ad-hoc\",\"data_rate_mbps\":6,"
                   "\"channel\":1,\"frequency_mhz\":2412,\"loss_model\":\"LogDistance\","
                   "\"exponent\":3,\"reference_loss_db\":40.05},\n\"rsus\":[";
        for (uint32_t i = 0; i < stations.size(); ++i)
        {
            const auto p = nodes.Get(i + 1)->GetObject<MobilityModel>()->GetPosition();
            results << (i ? "," : "") << "{\"rsu_id\":" << i + 1
                    << ",\"ip\":\"10.2.0." << i + 2 << "\",\"x_m\":" << p.x
                    << ",\"y_m\":" << p.y << ",\"z_m\":" << p.z
                    << ",\"generated_packets\":" << generated[i]
                    << ",\"socket_accepted_packets\":" << accepted[i]
                    << ",\"received_packets\":" << received[i]
                    << ",\"pdr\":";
            if (generated[i]) results << static_cast<double>(received[i]) / generated[i];
            else results << "null";
            results << ",\"mean_delay_ms\":";
            if (received[i])
            {
                results << totalDelay[i] / received[i];
            }
            else
            {
                results << "null";
            }
            results << "}";
        }
        results << "]\n}\n";
        summary.close();
        results.close();
        positions.close();
        association.close();
        packets.close();
        if (!summary || !results || !positions || !packets || !association)
        {
            throw std::runtime_error("Cannot flush output logs");
        }
    }
};

void ReceivePacket(Experiment* experiment, uint32_t rsu, Ptr<Socket> socket)
{
    experiment->Receive(rsu, socket);
}
} // namespace

int main(int argc, char* argv[])
{
    std::string outputDir;
    std::string progress = "summary";
    uint16_t bridgePort = 0;
    double duration = 300;
    double interval = 0.1;
    double txPowerDbm = 20;
    std::string rsuFile;
    uint32_t seed = 23423;
    uint64_t run = 1;
    CommandLine command(__FILE__);
    // Preserve whitespace in paths; the generic string parser reads a single token.
    command.AddValue("outputDir", "New run directory",
                     Callback<bool, const std::string&>([&outputDir](const std::string& value) {
                         outputDir = value;
                         return !value.empty();
                     }));
    command.AddValue("bridgePort", "Host TCP port for the live Python bridge", bridgePort);
    command.AddValue("duration", "Integer simulation duration, 5..390 seconds", duration);
    command.AddValue("interval", "UDP interval to the CURRENT nearest RSU only", interval);
    command.AddValue("txPowerDbm", "Wi-Fi transmit power in dBm", txPowerDbm);
    command.AddValue("rsuFile", "CSV roadside RSU layout",
                     Callback<bool, const std::string&>([&rsuFile](const std::string& value) {
                         rsuFile = value;
                         return !value.empty();
                     }));
    command.AddValue("seed", "ns-3 random seed", seed);
    command.AddValue("run", "ns-3 random run", run);
    command.AddValue("progress", "Console receive progress: summary, packets, or off", progress);
    command.Parse(argc, argv);
    try
    {
        if (!bridgePort || !std::filesystem::is_directory(outputDir)
            || !std::isfinite(duration) || duration < 5 || duration > 390
            || std::floor(duration) != duration || !std::isfinite(interval) || interval < 0.01
            || interval > 1 || !std::isfinite(txPowerDbm)
            || rsuFile.empty()
            || seed == 0 || seed >= 2147483647
            || (progress != "summary" && progress != "packets" && progress != "off"))
        {
            throw std::runtime_error("Invalid experiment arguments");
        }
        const std::filesystem::path output(outputDir);
        for (const auto& name : {"ns3_sync.csv", "packets.jsonl", "packet_summary.csv", "results.json"})
        {
            if (std::filesystem::exists(output / name))
            {
                throw std::runtime_error("Refusing to overwrite an existing run");
            }
        }
        const auto stations = ReadStations(rsuFile);
        RngSeedManager::SetSeed(seed);
        RngSeedManager::SetRun(run);
        // The car begins outside both coverage regions. The default 100 s DEAD
        // cache would suppress ARP rediscovery for this entire moving-car demo.
        Config::SetDefault("ns3::ArpCache::DeadTimeout", TimeValue(Seconds(1)));
        Bridge bridge(bridgePort);
        Experiment experiment;
        experiment.bridge = &bridge;
        experiment.stations = stations;
        experiment.generated.resize(stations.size());
        experiment.accepted.resize(stations.size());
        experiment.received.resize(stations.size());
        experiment.previousReceived.resize(stations.size());
        experiment.totalDelay.resize(stations.size());
        experiment.senders.resize(stations.size());
        experiment.durationSeconds = duration;
        experiment.progress = progress;
        experiment.nodes.Create(stations.size() + 1);
        experiment.positions.open(output / "ns3_sync.csv");
        experiment.packets.open(output / "packets.jsonl");
        experiment.association.open(output / "association.csv");
        experiment.association << std::setprecision(17) << "seq,ns3_time_s,from_rsu,to_rsu,distance_m,changed\n";
        if (!experiment.positions || !experiment.packets)
        {
            throw std::runtime_error("Cannot create output logs");
        }
        experiment.positions << std::setprecision(17)
            << "seq,ns3_time_s,sumo_time_s,time_offset_s,sumo_x_m,sumo_y_m,ns3_x_m,ns3_y_m,"
               "speed_mps,position_error_m,clock_error_s,prediction_correction_m";
        for (uint32_t i = 0; i < stations.size(); ++i)
            experiment.positions << ",distance_rsu" << i + 1 << "_m";
        for (uint32_t i = 0; i < stations.size(); ++i)
            experiment.positions << ",rsu" << i + 1 << "_tx,rsu" << i + 1 << "_rx";
        experiment.positions << ",serving_rsu\n";
        experiment.packets << std::setprecision(17);
        MobilityHelper carMobility;
        carMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
        carMobility.Install(experiment.nodes.Get(0));
        MobilityHelper stationMobility;
        stationMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        for (uint32_t i = 0; i < stations.size(); ++i)
        {
            stationMobility.Install(experiment.nodes.Get(i + 1));
            experiment.nodes.Get(i + 1)->GetObject<MobilityModel>()->SetPosition(stations[i]);
        }

        const std::string mode = "ErpOfdmRate6Mbps";
        Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue(mode));
        WifiHelper wifi;
        wifi.SetStandard(WIFI_STANDARD_80211g);
        wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue(mode),
                                    "ControlMode", StringValue(mode));
        YansWifiChannelHelper channel;
        channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
        channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel", "Exponent", DoubleValue(3),
                                   "ReferenceDistance", DoubleValue(1), "ReferenceLoss", DoubleValue(40.05));
        YansWifiPhyHelper phy;
        phy.SetChannel(channel.Create());
        phy.Set("TxPowerStart", DoubleValue(txPowerDbm));
        phy.Set("TxPowerEnd", DoubleValue(txPowerDbm));
        phy.Set("ChannelSettings", StringValue("{1, 20, BAND_2_4GHZ, 0}"));
        WifiMacHelper mac;
        mac.SetType("ns3::AdhocWifiMac");
        auto devices = wifi.Install(phy, mac, experiment.nodes);
        wifi.AssignStreams(devices, 0);
        InternetStackHelper internet;
        internet.Install(experiment.nodes);
        Ipv4AddressHelper addresses;
        addresses.SetBase("10.2.0.0", "255.255.255.0");
        auto interfaces = addresses.Assign(devices);
        auto& senders = experiment.senders;
        std::vector<Ptr<Socket>> receivers(stations.size());
        for (uint32_t i = 0; i < stations.size(); ++i)
        {
            receivers[i] = Socket::CreateSocket(experiment.nodes.Get(i + 1), UdpSocketFactory::GetTypeId());
            if (receivers[i]->Bind(InetSocketAddress(Ipv4Address::GetAny(), 9000)) != 0)
            {
                throw std::runtime_error("Cannot bind RSU UDP receiver");
            }
            receivers[i]->SetRecvCallback(MakeBoundCallback(&ReceivePacket, &experiment, i));
            senders[i] = Socket::CreateSocket(experiment.nodes.Get(0), UdpSocketFactory::GetTypeId());
            if (senders[i]->Connect(InetSocketAddress(interfaces.GetAddress(i + 1), 9000)) != 0)
            {
                throw std::runtime_error("Cannot set UDP destination");
            }
        }
        // Insert EVERY barrier first: at equal timestamps, mobility sync precedes packet sends.
        for (uint32_t seq = 0; seq <= static_cast<uint32_t>(duration); ++seq)
        {
            Simulator::Schedule(Seconds(seq), &Experiment::Sync, &experiment, seq);
        }
        for (uint32_t index = 0; 1 + index * interval < duration - 1 - 1e-9; ++index)
        {
            Simulator::Schedule(Seconds(1 + index * interval), &Experiment::SendNearest,
                                &experiment, index);
        }
        Simulator::Stop(Seconds(duration));
        Simulator::Run();
        if (bridge.Exchange("CLOSE\n") != "DONE")
        {
            throw std::runtime_error("Bridge did not acknowledge completion");
        }
        for (uint32_t i = 0; i < stations.size(); ++i)
        {
            senders[i]->Close();
            receivers[i]->Close();
        }
        experiment.WriteResults(output, duration, interval, txPowerDbm, seed, run);
        std::cout << "Live synchronization updates: " << experiment.syncCount << "\n";
        for (uint32_t i = 0; i < stations.size(); ++i)
        {
            std::cout << "RSU " << i + 1 << ": sent=" << experiment.generated[i]
                      << ", received=" << experiment.received[i] << "\n";
        }
        Simulator::Destroy();
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "Online experiment failed: " << error.what() << "\n";
        Simulator::Destroy();
        return 1;
    }
}


