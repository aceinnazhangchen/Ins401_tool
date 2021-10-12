#include <stdio.h>
#include <pcap.h>

#include <string.h>
#ifdef WIN32
#include <io.h>
#include <direct.h>
#else
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#endif

int makeDir(char* folderPath)
{
	int ret = -1;
#ifdef WIN32
	if (0 != _access(folderPath, 0))
	{
		ret = _mkdir(folderPath);
	}
#else
	if (-1 == access(folderPath, 0)) {
		ret = mkdir(folderPath, S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
	}
#endif
	return ret;
}

void createDirByFilePath(const char* filename, char* dirname) {
	char basename[64] = { 0 };
	strncpy(dirname, filename, strlen(filename));
	char* p = NULL;
#ifdef WIN32
	p = strrchr(dirname, '\\');
#else
	p = strrchr(dirname, '/');
#endif
	strcpy(basename, p);
	strcat(dirname, "_d");
	makeDir(dirname);
	strcat(dirname, basename);
}

#define LINE_LEN 16

#ifdef _WIN32
#include <tchar.h>
BOOL LoadNpcapDlls()
{
	_TCHAR npcap_dir[512];
	UINT len;
	len = GetSystemDirectory(npcap_dir, 480);
	if (!len) {
		fprintf(stderr, "Error in GetSystemDirectory: %x", GetLastError());
		return FALSE;
	}
	_tcscat_s(npcap_dir, 512, _T("\\Npcap"));
	if (SetDllDirectory(npcap_dir) == 0) {
		fprintf(stderr, "Error in SetDllDirectory: %x", GetLastError());
		return FALSE;
	}
	return TRUE;
}
#endif

//uint8_t output_mac[6] = { 0xE5,0xE6,0xE0,0x81,0x00,0x28 };	//37
//uint8_t output_mac[6] = { 0xDF,0xE6,0xE0,0x81,0x00,0x28 };	//31
//uint8_t output_mac[6] = { 0xDE,0xE6,0xE0,0x81,0x00,0x28 };	//30
//uint8_t output_mac[6] = { 0xD8,0xE6,0xE0,0x81,0x00,0x28 };	//24
//uint8_t output_mac[6] = { 0xDB,0xE6,0xE0,0x81,0x00,0x28 };	//27
uint8_t output_mac[6] = { 0xD5,0xE6,0xE0,0x81,0x00,0x28 };	//21
//uint8_t output_mac[6] = { 0xCA,0xE6,0xE0,0x81,0x00,0x28 };	//10

int main(int argc, char **argv)
{
	pcap_t *fp;
	char errbuf[PCAP_ERRBUF_SIZE];
	struct pcap_pkthdr *header;
	const u_char *pkt_data;
	u_int i=0;
	int res;
	
#ifdef _WIN32
	/* Load Npcap and its functions. */
	if (!LoadNpcapDlls())
	{
		fprintf(stderr, "Couldn't load Npcap\n");
		exit(1);
	}
#endif

	if(argc != 2)
	{	
		printf("usage: %s filename", argv[0]);
		return -1;

	}

	char* filename = argv[1];
	char dirname[256] = { 0 };
	createDirByFilePath(filename, dirname);
	char out_file_name[256] = { 0 };
	sprintf(out_file_name, "%s_%s", dirname, "log.log");
	FILE* f_log = fopen(out_file_name, "wb");
	if (f_log == NULL) {
		return -1;
	}
	sprintf(out_file_name, "%s_%s", dirname, "user.bin");
	FILE* f_bin = fopen(out_file_name, "wb");
	if (f_bin == NULL) {
		return -1;
	}

	/* Open the capture file */
	if ((fp = pcap_open_offline(filename,			// name of the device
						 errbuf					// error buffer
						 )) == NULL)
	{
		fprintf(stderr,"\nUnable to open the file %s.\n", argv[1]);
		return -1;
	}
	
	/* Retrieve the packets from the file */
	while((res = pcap_next_ex(fp, &header, &pkt_data)) >= 0)
	{
		/* print pkt timestamp and pkt len */
		//printf("%ld:%ld (%ld)\n", header->ts.tv_sec, header->ts.tv_usec, header->len);	
		uint8_t dst_mac[6] = { 0 };
		uint8_t src_mac[6] = { 0 };
		memcpy(dst_mac, pkt_data, 6);
		memcpy(src_mac, pkt_data+6, 6);
		char dst_mac_str[256] = { 0 };
		char src_mac_str[256] = { 0 };
		uint16_t msg_type = 0;
		memcpy(&msg_type, pkt_data + 14, 2);
		sprintf(dst_mac_str, "%02X:%02X:%02X:%02X:%02X:%02X", dst_mac[0], dst_mac[1], dst_mac[2], dst_mac[3], dst_mac[4], dst_mac[5]);
		sprintf(src_mac_str, "%02X:%02X:%02X:%02X:%02X:%02X", src_mac[0], src_mac[1], src_mac[2], src_mac[3], src_mac[4], src_mac[5]);

		if(memcmp(output_mac, src_mac,6) == 0){
			fprintf(f_log, "%ld:%06ld (%4ld:%4ld) src: %s dst: %s  msg_type:%04X\n", header->ts.tv_sec, header->ts.tv_usec, header->len, header->caplen, src_mac_str, dst_mac_str, msg_type);
			fwrite(pkt_data + 14, 1, header->caplen - 14, f_bin);
		}
		
		/* Print the packet */
/*		for (i=1; (i < header->caplen + 1 ) ; i++)
		{
			printf("%.2x ", pkt_data[i-1]);
			if ( (i % LINE_LEN) == 0) printf("\n");
		}
		
		printf("\n\n");	*/	
	}
	
	
	if (res == -1)
	{
		printf("Error reading the packets: %s\n", pcap_geterr(fp));
	}
	fclose(f_log);
	fclose(f_bin);
	pcap_close(fp);
	return 0;
}

