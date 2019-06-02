/*
 * Copyright (C) 2014 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL MATTHIAS
 * RINGWALD OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at 
 * contact@bluekitchen-gmbh.com
 *
 */

#define __BTSTACK_FILE__ "pan_lwip_http_server.c"

#include <stdio.h>

#include "btstack_config.h"
#include "btstack.h"

// network types
#define NETWORK_TYPE_IPv4       0x0800
#define NETWORK_TYPE_ARP        0x0806
#define NETWORK_TYPE_IPv6       0x86DD

static uint16_t bnep_cid            = 0;
static bd_addr_t bnep_addr;

static uint8_t pan_sdp_record[220];

static btstack_packet_callback_registration_t hci_event_callback_registration;

static int pan_bnep_enabled;

// outgoing network packet
static const uint8_t * network_buffer;
static uint16_t        network_buffer_len;


/* @section Main application configuration
 *
 * @text In the application configuration, L2CAP and BNEP are initialized and a BNEP service, for server mode,
 * is registered, before the Bluetooth stack gets started, as shown in Listing PanuSetup.
 */

/* LISTING_START(PanuSetup): Panu setup */
static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void network_send_packet_callback(const uint8_t * packet, uint16_t size);

/* LISTING_END */

/*
 * @section Packet Handler
 * 
 * @text The packet handler responds to various HCI Events.
 */

/* LISTING_START(packetHandler): Packet Handler */
static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
/* LISTING_PAUSE */
    UNUSED(channel);

    uint8_t   event;
    bd_addr_t local_addr;
    bd_addr_t event_addr;
    uint16_t  uuid_source;
    uint16_t  uuid_dest;
    uint16_t  mtu;    
  
    /* LISTING_RESUME */
    switch (packet_type) {
		case HCI_EVENT_PACKET:
            event = hci_event_packet_get_type(packet);
            switch (event) {            

                /* LISTING_PAUSE */
                case HCI_EVENT_PIN_CODE_REQUEST:
					// inform about pin code request
                    printf("Pin code request - using '0000'\n");
                    hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                    gap_pin_code_response(event_addr, "0000");
					break;

                case HCI_EVENT_USER_CONFIRMATION_REQUEST:
                    // inform about user confirmation request
                    printf("SSP User Confirmation Auto accept\n");
                    hci_event_user_confirmation_request_get_bd_addr(packet, event_addr);
                    break;

                /* LISTING_RESUME */

                /* @text BNEP_EVENT_CHANNEL_OPENED is received after a BNEP connection was established or 
                 * or when the connection fails. The status field returns the error code.
                 * 
                 * The TAP network interface is then configured. A data source is set up and registered with the 
                 * run loop to receive Ethernet packets from the TAP interface.
                 *
                 * The event contains both the source and destination UUIDs, as well as the MTU for this connection and
                 * the BNEP Channel ID, which is used for sending Ethernet packets over BNEP.
                 */  
				case BNEP_EVENT_CHANNEL_OPENED:
                    if (bnep_event_channel_opened_get_status(packet)) {
                        printf("BNEP channel open failed, status %02x\n", bnep_event_channel_opened_get_status(packet));
                    } else {
                        bnep_cid    = bnep_event_channel_opened_get_bnep_cid(packet);
                        uuid_source = bnep_event_channel_opened_get_source_uuid(packet);
                        uuid_dest   = bnep_event_channel_opened_get_destination_uuid(packet);
                        mtu         = bnep_event_channel_opened_get_mtu(packet);
                        memcpy(&bnep_addr, &packet[11], sizeof(bd_addr_t));
                        printf("BNEP connection open succeeded to %s source UUID 0x%04x dest UUID: 0x%04x, max frame size %u\n", bd_addr_to_str(bnep_addr), uuid_source, uuid_dest, mtu);

                        /* Setup network interface */
                        gap_local_bd_addr(local_addr);
                        btstack_network_up(local_addr);
                        printf("Network Interface '%s' activated\n", btstack_network_get_name());
                    }
					break;
                
                /* @text If there is a timeout during the connection setup, BNEP_EVENT_CHANNEL_TIMEOUT will be received
                 * and the BNEP connection  will be closed
                 */     
                case BNEP_EVENT_CHANNEL_TIMEOUT:
                    printf("BNEP channel timeout! Channel will be closed\n");
                    break;

                /* @text BNEP_EVENT_CHANNEL_CLOSED is received when the connection gets closed.
                 */
                case BNEP_EVENT_CHANNEL_CLOSED:
                    printf("BNEP channel closed\n");
                    bnep_cid = 0;

                    // discard outgoing packet
                    if (network_buffer_len){
                        network_buffer_len = 0;
                        btstack_network_packet_sent();
                    }

                    btstack_network_down();
                    break;

                /* @text BNEP_EVENT_CAN_SEND_NOW indicates that a new packet can be send. This triggers the send of a 
                 * stored network packet. The tap datas source can be enabled again
                 */
                case BNEP_EVENT_CAN_SEND_NOW:
                    if (network_buffer_len > 0) {
                        bnep_send(bnep_cid, (uint8_t*) network_buffer, network_buffer_len);
                        network_buffer_len = 0;
                        btstack_network_packet_sent();
                    }
                    break;
                    
                default:
                    break;
            }
            break;

        /* @text Ethernet packets from the remote device are received in the packet handler with type BNEP_DATA_PACKET.
         * It is forwarded to the TAP interface.
         */
        case BNEP_DATA_PACKET:
            // Write out the ethernet frame to the network interface
            btstack_network_process_packet(packet, size);
            break;            
            
        default:
            break;
    }
}
/* LISTING_END */

/*
 * @section Network packet handler
 * 
 * @text A pointer to the network packet is stored and a BNEP_EVENT_CAN_SEND_NOW requested
 */

/* LISTING_START(networkPacketHandler): Network Packet Handler */
static void network_send_packet_callback(const uint8_t * packet, uint16_t size){
    if (bnep_cid){
        network_buffer = packet;
        network_buffer_len = size;
        bnep_request_can_send_now_event(bnep_cid);
    } else {
        btstack_network_packet_sent();
    }
}
/* LISTING_END */

static void pan_bnep_setup(void){

    // Initialize BNEP
    bnep_init();

    // Init SDP
    sdp_init();
    memset(pan_sdp_record, 0, sizeof(pan_sdp_record));
    uint16_t network_packet_types[] = { NETWORK_TYPE_IPv4, NETWORK_TYPE_ARP, 0};    // 0 as end of list
    // Minimum L2CAP MTU for bnep is 1691 bytes
    bnep_register_service(packet_handler, BLUETOOTH_SERVICE_CLASS_NAP, 1691);
    // NAP Network Access Type: Other, 1 MB/s
    pan_create_nap_sdp_record(pan_sdp_record, sdp_create_service_record_handle(), network_packet_types, NULL, NULL, BNEP_SECURITY_NONE, PAN_NET_ACCESS_TYPE_OTHER, 1000000, NULL, NULL);
    sdp_register_service(pan_sdp_record);
    printf("SDP service record size: %u\n", de_get_len((uint8_t*) pan_sdp_record));

	// Discoverable
	// Set local name with a template Bluetooth address, that will be automatically
	// replaced with a actual address once it is available, i.e. when BTstack boots
	// up and starts talking to a Bluetooth module.
	gap_set_local_name("PAN BNEP 00:00:00:00:00:00");
	gap_discoverable_control(1);
	gap_set_class_of_device(0x2540);

    // Initialize network interface
    btstack_network_init(&network_send_packet_callback);

    pan_bnep_enabled = 1;
}


// lwip code

// DHCP Server
#include "lwip/opt.h"
#include "lwip/tcpip.h"

#if 0

// DHCP

#define NUM_DHCP_ENTRY 3

static dhcp_entry_t entries[NUM_DHCP_ENTRY] =
{
    /* mac    ip address        subnet mask        lease time */
    { {0}, {192, 168, 7, 2}, {255, 255, 255, 0}, 24 * 60 * 60 },
    { {0}, {192, 168, 7, 3}, {255, 255, 255, 0}, 24 * 60 * 60 },
    { {0}, {192, 168, 7, 4}, {255, 255, 255, 0}, 24 * 60 * 60 }
};

static dhcp_config_t dhcp_config =
{
    {192, 168, 7, 1}, 67, /* server address, port */
    {192, 168, 7, 1},     /* dns server */
    "stm",                /* dns suffix */
    NUM_DHCP_ENTRY,       /* num entry */
    entries               /* entries */
};

#endif

#include "lwip/init.h"

/*!
 * @brief Initializes lwIP stack.
 */
static void network_setup(void){
    printf("lwIP version: " LWIP_VERSION_STRING "\n");

#if BYTE_ORDER == LITTLE_ENDIAN
    // big endian detection not supported by build configuration
    if (btstack_is_big_endian()){
        printf("lwIP configured for little endian, but running on big endian. Please set BYTE_ORDER to BIG_ENDIAN in lwiopts.h\n");
        while (1);
    }
#endif

    // init lwIP stack
    lwip_init();

#if 0
    // start DHCP Server
    dhserv_init(&dhcp_config);

    // start HTTP Server
    lwip_httpsserv_init();
#endif
}

// end of network code


int btstack_main(int argc, const char * argv[]);
int btstack_main(int argc, const char * argv[]){

    (void)argc;
    (void)argv;

    // register for HCI events
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // Initialize L2CAP
    l2cap_init();

    // setup Classic PAN NAP Service
    pan_bnep_setup();

    // setup lwIP, HTTP, DHCP
    network_setup();

    // Turn on the device 
    hci_power_control(HCI_POWER_ON);
    return 0;
}

/* EXAMPLE_END */
