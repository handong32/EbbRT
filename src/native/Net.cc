//          Copyright Boston University SESA Group 2013 - 2014.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#include "Net.h"

void ebbrt::NetworkManager::Init() {}

void ebbrt::NetworkManager::ApInit()
{
  network_manager->CpuInit();
}

void ebbrt::NetworkManager::CpuInit() {
  interface_->CpuInit();
}

ebbrt::NetworkManager::Interface&
ebbrt::NetworkManager::NewInterface(EthernetDevice& ether_dev) {
  interface_.reset(new Interface(ether_dev));
  return *interface_;
}

void ebbrt::NetworkManager::Interface::Receive(std::unique_ptr<MutIOBuf> buf, uint64_t rxflag) {
  auto packet_len = buf->ComputeChainDataLength();

  //ebbrt::kprintf("%s packet_len=%d \n", __FUNCTION__, packet_len);
  
  // Drop packets that are too small
  if (packet_len <= sizeof(EthernetHeader)) {
    ebbrt::kprintf("\t%s a\n", __FUNCTION__);
    return;
  }
  auto dp = buf->GetMutDataPointer();
  
  auto& eth_header = dp.Get<EthernetHeader>();
  //ebbrt::kprintf("\t%s b\n", __FUNCTION__);
  //ebbrt::kprintf("sizeof etherheader: %d addr:%p\n", sizeof(EthernetHeader), dp.Data());

  //ebbrt::kprintf("length = %d ischain = %d\n", buf->Length(), buf->IsChained());
  
  buf->Advance(sizeof(EthernetHeader));
  //ebbrt::kprintf("buf->Advance -> %p\n", buf->MutData());
  //ebbrt::kprintf("\t%s c\n", __FUNCTION__);
  
  switch (ntohs(eth_header.type)) {
  case kEthTypeIp: {
    /*
    ebbrt::kprintf("eth type ip %d\n", eth_header.type);
    ebbrt::kprintf("dst: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", eth_header.dst[0],
      eth_header.dst[1], eth_header.dst[2], eth_header.dst[3], eth_header.dst[4], 
      eth_header.dst[5]);
    ebbrt::kprintf("src: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", eth_header.src[0],
      eth_header.src[1], eth_header.src[2], eth_header.src[3], eth_header.src[4], 
      eth_header.src[5]);
    */
    // auto dp2 = buf->GetMutDataPointer();
    // auto tbuf2 = dp2.Data();
    
    // ebbrt::kprintf("tbuf2: %p %p\n", tbuf2, buf->MutData());
    
    // for(int i = 0; i < 20; i++)
    // {
    //   ebbrt::kprintf("0x%02X ", tbuf2[i]);
    // }
    // ebbrt::kprintf("\n");
    
    //ebbrt::kprintf("eth type ip\n");
    ReceiveIp(eth_header, std::move(buf), rxflag);
    break;
  }
  case kEthTypeArp: {
    //ebbrt::kprintf("eth type arp\n");
    ReceiveArp(eth_header, std::move(buf));
    break;
  }
    /*default: {
    //ebbrt::kprintf("\t%s d\n", __FUNCTION__);
    ebbrt::kabort("%s error\n", __FUNCTION__);
    }*/
    
  }

  //ebbrt::kprintf("receive done\n\n\n");
}

const ebbrt::EthernetAddress& ebbrt::NetworkManager::Interface::MacAddress() {
  return ether_dev_.GetMacAddress();
}

void ebbrt::NetworkManager::Interface::Send(std::unique_ptr<IOBuf> b,
                                            PacketInfo pinfo) {
  ether_dev_.Send(std::move(b), std::move(pinfo));
}
