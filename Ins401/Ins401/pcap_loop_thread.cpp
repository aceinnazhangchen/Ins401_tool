#include "pcap_loop_thread.h"
#include <qdebug.h>
#include "common_function.h"
#include "devices_manager.h"

pcap_loop_thread::pcap_loop_thread(QObject *parent)
	: QThread(parent)
	, m_isStop(false)
	, adhandle(NULL)
{
	card_name.clear();
}

pcap_loop_thread::~pcap_loop_thread()
{
}

void pcap_loop_thread::run()
{
	m_isStop = false;
	if (open_ethernet() == false) return;

	int res;
	struct pcap_pkthdr *header;
	const u_char *pkt_data;
	emit devices_manager::Instance().sgnLog("running...");
	uint64_t src_mac_num = 0;
	while ((res = pcap_next_ex(adhandle, &header, &pkt_data)) >= 0)
	{
		//emit sgnLog(QString("test"));
		if (m_isStop) break;
		if (res == 0) continue; /* Timeout elapsed */
		memcpy(&src_mac_num, pkt_data+ MAC_ADDRESS_LEN, MAC_ADDRESS_LEN);
		if (src_mac_num == devices_manager::Instance().src_mac_num) continue;
		recv_pack(header, pkt_data);

	}
	if (res == -1) {
		emit devices_manager::Instance().sgnLog(QString::asprintf("Error reading the packets: %s", pcap_geterr(adhandle)));
	}

	pcap_close(adhandle);
	adhandle = NULL;
	emit devices_manager::Instance().sgnLog("closed");
}

void pcap_loop_thread::stop()
{
	m_isStop = true;
}

void pcap_loop_thread::set_dev_name(char * name)
{
	card_name = name;
}

bool pcap_loop_thread::open_ethernet()
{
	char errbuf[PCAP_ERRBUF_SIZE];
	if ((adhandle = pcap_open_live(card_name.c_str(),	// name of the device
		65536,			// portion of the packet to capture. 
						// 65536 grants that the whole packet will be captured on all the MACs.
		1,				// promiscuous mode (nonzero means promiscuous)
		1000,			// read timeout
		errbuf			// error buffer
	)) == NULL)
	{
		emit devices_manager::Instance().sgnLog(QString::asprintf("Unable to open the adapter. %s is not supported by Npcap", card_name.c_str()));
		return false;
	}
	return true;
}

void pcap_loop_thread::send_pack(u_char* buffer, uint32_t nlen)
{
	if (adhandle == NULL) return;
	if ((pcap_datalink(adhandle) == DLT_EN10MB)) {

		if (pcap_sendpacket(adhandle,			// Adapter
			(u_char*)buffer,					// buffer with the packet
			nlen								// size
		) != 0)
		{
			emit devices_manager::Instance().sgnLog(QString::asprintf("Error sending the packet: %s", pcap_geterr(adhandle)));
			return;
		}
	}
}

void pcap_loop_thread::recv_pack(pcap_pkthdr* header, const u_char* buffer)
{
	if (header == NULL || buffer == NULL) return;

	struct tm *ltime;
	char timestr[16];
	time_t local_tv_sec;
	/* convert the timestamp to readable format */
	local_tv_sec = header->ts.tv_sec;
	ltime = localtime(&local_tv_sec);
	strftime(timestr, sizeof timestr, "%H:%M:%S", ltime);

	if (header->len > MAC_ADDRESS_LEN*2){
		if (devices_manager::Instance().log_flag) {
			devices_manager::Instance().log_data((const uint8_t*)buffer, header->len);
		}
		app_packet_t pak = { 0 };
		memcpy(&pak, buffer, header->len);
		devices_manager::Instance().pack_from_little_endian(pak);
		if (devices_manager::Instance().fitler_pack(pak) && devices_manager::Instance().fitler_mac(pak)) {
			//emit devices_manager::Instance().sgnLog(QString::asprintf("%s.%.6d len:%3d,caplen:%3d", timestr, header->ts.tv_usec, header->len, header->caplen));		
			devices_manager::Instance().dispatch_pak(pak);
		}
	}
	else
	{
		emit devices_manager::Instance().sgnLog(QString::asprintf("%s,%.6d len:%3d,caplen:%3d",
			timestr, header->ts.tv_usec, header->len, header->caplen));
	}
}