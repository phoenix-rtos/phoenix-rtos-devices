#include "helper.h"


typedef struct {
	int bitNum;
	char *bitName;
	char *bitInfo;
} stm32n6_libusbHelper;


static stm32n6_libusbHelper gintsts_bits_data[] = {
	{ 31, "WKUPINT", "Resume/remote wake-up detected interrupt" },
	{ 30, "SRQINT", "Session request/new session detected interrupt" },
	{ 28, "CIDSCHG", "Connector ID status change" },
	{ 29, "DISCINT", "Disconnect detected interrupt" },
	{ 27, "LPMINT", "LPM interrupt" },
	{ 26, "PTXFE", "Periodic Tx FIFO empty" },
	{ 25, "HCINT", "Host channels interrupt" },
	{ 24, "HPRTINT", "Host port interrupt" },
	{ 23, "RSTDET", "Reset detected interrupt" },
	{ 22, "DATAFSUSP", "Data fetch suspended" },
	{ 21, "INCOMPISOOUT", "Incomplete isochronous OUT transfer" },
	{ 20, "IISOIXFR", "Incomplete isochronous IN transfer" },
	{ 19, "OEPINT", "OUT endpoint interrupt" },
	{ 18, "IEPINT", "IN endpoint interrupt" },
	{ 15, "EOPF", "End of periodic frame interrupt" },
	{ 14, "ISOODRP", "Isochronous OUT packet dropped interrupt" },
	{ 13, "ENUMDNE", "Enumeration done" },
	{ 12, "USBRST", "USB reset" },
	{ 11, "USBSUSP", "USB suspend" },
	{ 10, "ESUSP", "Early suspend" },
	{ 7, "GONAKEFF", "Global OUT NAK effective" },
	{ 6, "GINAKEFF", "Global IN non-periodic NAK effective" },
	{ 5, "NPTXFE", "Non-periodic Tx FIFO empty" },
	{ 4, "RXFLVL", "Rx FIFO non-empty" },
	{ 3, "SOF", "Start of frame" },
	{ 2, "OTGINT", "OTG interrupt" },
	{ 1, "MMIS", "Mode mismatch interrupt" }
};


static stm32n6_libusbHelper doepctl0_bits_data[] = {
	{ 31, "EPENA", "Endpoint enable" },
	{ 30, "EPDIS", "Endpoint disable" },
	{ 27, "SNAK", "Set NAK" },
	{ 26, "CNAK", "Clear NAK" },
	{ 21, "STALL", "STALL handshake" },
	{ 20, "SNPM", "Snoop mode" },
	{ 17, "NAKSTS", "NAK status" },
	{ 15, "USBAEP", "USB active endpoint" },
	{ 1, "MPSIZ[1:0]", "Maximum packet size" },
	{ 0, "MPSIZ[1:0]", "Maximum packet size" }
};


void helper_showRegisterInfo(uint32_t regVal, int registerH)
{
	switch (registerH) {
		case HELPER_GINSTSTS:
			for (int i = 0; i < 32; i++) {
				if ((regVal >> gintsts_bits_data[i].bitNum) & 1) {
					printf("[HELPER]: bit: %s, info: %s\n", gintsts_bits_data[i].bitName, gintsts_bits_data[i].bitInfo);
				}
			}
			break;
		case HELPER_DOEPCTL0:
			for (int i = 0; i < 32; i++) {
				if ((regVal >> doepctl0_bits_data[i].bitNum) & 1) {
					printf("[HELPER]: bit: %s, info: %s\n", doepctl0_bits_data[i].bitName, doepctl0_bits_data[i].bitInfo);
				}
			}
			break;
		default:
			break;
	}
}
