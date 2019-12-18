//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#include "Net.h"

void ebbrt::NetworkManager::Init() {}

ebbrt::NetworkManager::Interface&
ebbrt::NetworkManager::NewInterface(EthernetDevice& ether_dev) {
  interface_.reset(new Interface(ether_dev));
  return *interface_;
}

void ebbrt::NetworkManager::Interface::Receive(std::unique_ptr<MutIOBuf> buf,
                                               uint64_t rxflag) {
  auto packet_len = buf->ComputeChainDataLength();

  // Drop packets that are too small
  if (packet_len <= sizeof(EthernetHeader)) {
    ebbrt::kprintf("packet_len=%d too small (less than EthernetHeader)\n", packet_len);
    return;
  }

  auto dp = buf->GetMutDataPointer();
  auto& eth_header = dp.Get<EthernetHeader>();
  buf->Advance(sizeof(EthernetHeader));

  switch (ntohs(eth_header.type)) {
  case kEthTypeIp: {
    ReceiveIp(eth_header, std::move(buf), rxflag);
    break;
  }
  case kEthTypeArp: {
    ReceiveArp(eth_header, std::move(buf));
    break;
  }
  default: {
    ebbrt::kprintf("NetworkManager::Interface::Receive(): Unknown eth_header.type=0x%X packet_len=%u\n", ntohs(eth_header.type), packet_len);
    ebbrt::kabort("NetworkManager::Interface::Receive()\n");
    /*for (int i = 0; i < (int)packet_len; i+=8) {
      if (i+8 < (int)packet_len) {
	ebbrt::kprintf("%02X%02X%02X%02X%02X%02X%02X%02X\n", p1[i], p1[i+1], p1[i+2], p1[i+3], p1[i+4], p1[i+5], p1[i+6], p1[i+7]);
      } else {
	for(int j = i; j < (int)packet_len; j++) {
	  ebbrt::kprintf("%02X\n", p1[j]);
	}
      }
      }*/
  }
  }
}

const ebbrt::EthernetAddress& ebbrt::NetworkManager::Interface::MacAddress() {
  return ether_dev_.GetMacAddress();
}

void ebbrt::NetworkManager::Interface::Send(std::unique_ptr<IOBuf> b,
                                            PacketInfo pinfo) {
  ether_dev_.Send(std::move(b), std::move(pinfo));
}

void ebbrt::NetworkManager::Config(std::string s, uint32_t v) {
  interface_->Config(std::move(s), v);
}

void ebbrt::NetworkManager::Interface::Config(std::string s, uint32_t v) {
  ether_dev_.Config(std::move(s), v);
}
