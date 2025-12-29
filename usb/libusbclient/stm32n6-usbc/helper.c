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

static stm32n6_libusbHelper doepint_bits_data[] = {
	{ 15, "STPKTRX", "Setup packet received" },
	{ 14, "NYET", "NYET interrupt" },
	{ 13, "NAK", "NAK input" },
	{ 12, "BERR", "Babble error interrupt" },
	{ 8, "OUTPKTERR", "OUT packet error" },
	{ 6, "B2BSTUP", "Back-to-back SETUP packets received" },
	{ 5, "STSPHSRX", "Status phase received for control write" },
	{ 4, "OTEPDIS", "OUT token received when endpoint disabled" },
	{ 3, "STUP", "SETUP phase done" },
	{ 2, "AHBERR", "AHB error" },
	{ 1, "EPDISD", "Endpoint disabled interrupt" },
	{ 0, "XFRC", "Transfer completed interrupt" }
};

static stm32n6_libusbHelper diepint_bits_data[] = {
	{ 13, "NAK", "NAK input" },
	{ 11, "PKTDRPSTS", "Packet dropped status" },
	{ 8, "TXFIFOUDRN", "Transmit Fifo Underrun (TxfifoUndrn)" },
	{ 7, "TXFE", "Transmit FIFO empty" },
	{ 6, "INEPNE", "IN endpoint NAK effective" },
	{ 5, "INEPNM", "IN token received with EP mismatch" },
	{ 4, "ITTXFE", "IN token received when Tx FIFO is empty" },
	{ 3, "TOC", "Timeout condition" },
	{ 2, "AHBERR", "AHB error" },
	{ 0, "XFRC", "Transfer completed interrupt" }
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

static stm32n6_libusbHelper diepctl0_bits_data[] = {
	{ 13, "NAK", "NAK input" },
	{ 12, "Reserved, must be kept at reset value." },
	{ 11, "PKTDRPSTS", "Packet dropped status" },
	{ 10, "Reserved, must be kept at reset value." },
	{ 9, "Reserved, must be kept at reset value." },
	{ 8, "TXFIFOUDRN: Transmit Fifo Underrun (TxfifoUndrn)" },
	{ 7, "TXFE", "Transmit FIFO empty" },
	{ 6, "INEPNE", "IN endpoint NAK effective" },
	{ 5, "INEPNM", "IN token received with EP mismatch" },
	{ 4, "ITTXFE", "IN token received when Tx FIFO is empty" },
	{ 3, "TOC", "Timeout condition" },
	{ 2, "AHBERR", "AHB error" },
	{ 1, "EPDISD", "Endpoint disabled interrupt" },
	{ 0, "XFRC", "Transfer completed interrupt" }
};


void helper_showRegisterInfo(uint32_t regVal, int registerH)
{
	switch (registerH) {
		case HELPER_GINSTSTS:
			for (int i = 0; i < sizeof(gintsts_bits_data) / sizeof(gintsts_bits_data[0]); i++) {
				if ((regVal >> gintsts_bits_data[i].bitNum) & 1) {
					printf("	[GINSTSTS]: bit: %s, info: %s\n", gintsts_bits_data[i].bitName, gintsts_bits_data[i].bitInfo);
				}
			}
			break;

		case HELPER_DOEPINT:
			for (int i = 0; i < sizeof(doepint_bits_data) / sizeof(doepint_bits_data[0]); i++) {
				if ((regVal >> doepint_bits_data[i].bitNum) & 1) {
					printf("	[DOEPINT]: bit: %s, info: %s\n", doepint_bits_data[i].bitName, doepint_bits_data[i].bitInfo);
				}
			}
			break;
		case HELPER_DIEPINT:

			for (int i = 0; i < sizeof(diepint_bits_data) / sizeof(diepint_bits_data[0]); i++) {
				if ((regVal >> diepint_bits_data[i].bitNum) & 1) {
					printf("	[DIEPINT]: bit: %s, info: %s\n", diepint_bits_data[i].bitName, diepint_bits_data[i].bitInfo);
				}
			}
			break;

		case HELPER_DOEPCTL0:

			for (int i = 0; i < sizeof(doepctl0_bits_data) / sizeof(doepctl0_bits_data[0]); i++) {
				if ((regVal >> doepctl0_bits_data[i].bitNum) & 1) {
					printf("	[DOEPCTL0]: bit: %s, info: %s\n", doepctl0_bits_data[i].bitName, doepctl0_bits_data[i].bitInfo);
				}
			}
			break;

		case HELPER_DIEPCTL0:
			for (int i = 0; i < sizeof(diepctl0_bits_data) / sizeof(diepctl0_bits_data[0]); i++) {
				if ((regVal >> diepctl0_bits_data[i].bitNum) & 1) {
					printf("	[DIEPCTL0]: bit: %s, info: %s\n", diepctl0_bits_data[i].bitName, diepctl0_bits_data[i].bitInfo);
				}
			}
			break;
		default:
			break;
	}
}
