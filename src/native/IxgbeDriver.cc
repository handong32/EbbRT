//          Copyright Boston University SESA Group 2013 - 2018.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
#include "IxgbeDriver.h"

#include "../Align.h"
#include "../StaticIOBuf.h"
#include "../UniqueIOBuf.h"
#include "Clock.h"
#include "Debug.h"
#include "EventManager.h"
#include "Fls.h"
#include "Ixgbe.h"
#include "Net.h"
#include "Pfn.h"

#include <atomic>
#include <cinttypes>
#include <mutex>

struct IxgbeLog ixgbe_logs[16];

void dumpPacketContents(uint8_t* p1, uint64_t len) {
  uint64_t i, j;
  
  ebbrt::kprintf_force("dumpPacketContents() len=%u\n", len);
  for (i = 0; i < len; i+=8) {
    if (i+8 < len) {
      ebbrt::kprintf("%02X%02X%02X%02X%02X%02X%02X%02X\n", p1[i], p1[i+1], p1[i+2], p1[i+3], p1[i+4], p1[i+5], p1[i+6], p1[i+7]);
    } else {
      for(j=i;j<len;j++) {
	ebbrt::kprintf("%02X\n", p1[j]);
      }
      i = len;
      return;
    }
  }
}

ebbrt::IxgbeDriver* ebbrt::ixgbe_dev;

void ebbrt::IxgbeDriver::Create(pci::Device& dev) {
  //auto ixgbe_dev = new IxgbeDriver(dev);
  ixgbe_dev = new IxgbeDriver(dev);
  
  // physical device bringup
  ixgbe_dev->Init();

  ixgbe_dev->ebb_ =
      IxgbeDriverRep::Create(ixgbe_dev, ebb_allocator->AllocateLocal());

  // only even core numbers
  if(static_cast<int>(Cpu::Count()) > 1) {
    kassert(static_cast<int>(Cpu::Count()) % 2 == 0);
  }

  // initialize per core rx and tx queues
  for (size_t i = 0; i < Cpu::Count(); i++) {
    ixgbe_dev->SetupMultiQueue(i);
  }

  ixgbe_dev->FinishSetup();

  // TODO remove?
  ebbrt::clock::SleepMilli(200);
  ebbrt::kprintf("82599 initialze complete\n");

  memset(ixgbe_logs, 0, sizeof(ixgbe_logs));
}

const ebbrt::EthernetAddress& ebbrt::IxgbeDriver::GetMacAddress() {
  return mac_addr_;
}

std::string ebbrt::IxgbeDriver::ReadNic() {    
  uint32_t i = static_cast<uint32_t>(Cpu::GetMine());
  return ixgmq[i]->str_stats.str();
}

void* ebbrt::IxgbeDriver::ReadItr() {
  return static_cast<void*>(ixgmq[1]->itr_stats.data());
}

//std::vector<ITR_STATS> ebbrt::IxgbeDriver::ReadItrStats(uint32_t v) {
//  return ixgmq[v]->itr_stats;
//}

void ebbrt::IxgbeDriver::Config(std::string s, uint32_t v) {
  uint32_t i = static_cast<uint32_t>(Cpu::GetMine());
  //uint32_t c = 0;
  
  if(s == "rx_usecs") {
    ixgmq[i]->itr_val = v;
    ebbrt::kprintf_force("rx-usecs = %u\n", ixgmq[i]->itr_val*2);
    WriteEitr(i, (ixgmq[i]->itr_val << 3) | IXGBE_EITR_CNT_WDIS);
  } else if(s == "rapl") {
    if(i == 0 || i == 1) {
      ixgmq[i]->powerMeter.SetLimit(v);
    }
  } else if(s == "start_perf") {
    ebbrt::kprintf_force("start_perf %d\n", i);
    for(int j=0;j < 100;j++) {
      ixgmq[i]->rx_desc_counts[j] = 0;
      ixgmq[i]->tx_desc_counts[j] = 0;
    }

    ixgmq[i]->time_us = 0;
    ixgmq[i]->totalNrg = 0;
    //if(i == 0 || i == 1) {
    auto d = ebbrt::clock::Wall::Now().time_since_epoch();
    ixgmq[i]->time_us = std::chrono::duration_cast<std::chrono::microseconds>(d).count();
    ixgmq[i]->powerMeter.Start();
      //}
        
    //ebbrt::kprintf_force("%d: %d %d\n", i, ixgmq[i]->tx_desc_counts.size(), ixgmq[i]->rx_desc_counts.size());
    //ixgmq[i]->stat_num_recv = 0;
    
    ixgmq[i]->totalCycles = 0;
    ixgmq[i]->totalIns = 0;
    ixgmq[i]->totalLLCmisses = 0;
    
    ixgmq[i]->fireCount = 0;
      
    ixgmq[i]->perfCycles.Start();
    ixgmq[i]->perfInst.Start();
    ixgmq[i]->perfLLC_miss.Start();    

    ebb_->StartTimer();
    
  } else if(s == "stop_perf") {
    //if(i == 0 || i == 1) {
    ixgmq[i]->powerMeter.Stop();
    ixgmq[i]->totalNrg += ixgmq[i]->powerMeter.Read();
    auto d = ebbrt::clock::Wall::Now().time_since_epoch();
    auto endt = std::chrono::duration_cast<std::chrono::microseconds>(d).count();
    ixgmq[i]->totalTime = ((double)(endt - (ixgmq[i]->time_us)) / 1000000.0);
    ixgmq[i]->totalPower = ixgmq[i]->totalNrg / ixgmq[i]->totalTime;
      //}
    
    ixgmq[i]->perfCycles.Stop();
    ixgmq[i]->perfInst.Stop();
    ixgmq[i]->perfLLC_miss.Stop();
    
    // accumulate counters
    ixgmq[i]->totalCycles += static_cast<uint64_t>(ixgmq[i]->perfCycles.Read());
    ixgmq[i]->totalIns += static_cast<uint64_t>(ixgmq[i]->perfInst.Read());
    ixgmq[i]->totalLLCmisses += static_cast<uint64_t>(ixgmq[i]->perfLLC_miss.Read());
    //ixgmq[i]->totalInterrupts = ixgmq[i]->stat_num_recv;
      
    //if(i == 0 || i == 1) {
    //ixgmq[i]->totalNrg += ixgmq[i]->powerMeter.Read();
    //auto d = ebbrt::clock::Wall::Now().time_since_epoch();
    //auto endt = std::chrono::duration_cast<std::chrono::microseconds>(d).count();
    //ixgmq[i]->totalTime = ((double)(endt - (ixgmq[i]->time_us)) / 1000000.0);
    //ixgmq[i]->totalPower = ixgmq[i]->totalNrg / ixgmq[i]->totalTime;
    //ebbrt::kprintf_force("Core %u: cycles=%llu ins=%llu llc=%llu energy=%.2lfJ totalTime=%.2f secs Power (Watts): %.2lf\n", i, ixgmq[i]->totalCycles, ixgmq[i]->totalIns, ixgmq[i]->totalLLCmisses, ixgmq[i]->totalNrg, ixgmq[i]->totalTime, ixgmq[i]->totalPower);
      //}
    ixgmq[i]->perfCycles.Clear();
    ixgmq[i]->perfInst.Clear();
    ixgmq[i]->perfLLC_miss.Clear();
    //ixgmq[i]->stat_num_recv = 0;
    
    ebb_->StopTimer();
  } else if(s == "print") {
    double ttime, tnrg;
    ttime = tnrg = 0.0;    
    ttime = ixgmq[0]->totalTime > ixgmq[1]->totalTime ? ixgmq[0]->totalTime : ixgmq[1]->totalTime;
    for(uint32_t i = 0; i < static_cast<uint32_t>(Cpu::Count()); i++) {
      tnrg += ixgmq[i]->totalNrg;
    }	
    ebbrt::kprintf_force("TIME=%.2fs\n", ttime);
    ebbrt::kprintf_force("WATTS=%.2f\n", tnrg/ttime);
    
    /*uint64_t cycs, ins, llc, nints;         
    cycs = ins = llc = nints = 0;

    for(uint32_t i = 0; i < static_cast<uint32_t>(Cpu::Count()); i++) {
      cycs += ixgmq[i]->totalCycles;
      ins += ixgmq[i]->totalIns;
      llc += ixgmq[i]->totalLLCmisses;
      tnrg += ixgmq[i]->totalNrg;
      nints += ixgmq[i]->totalInterrupts;
    }
    
    ixgmq[i]->str_stats.str("");
    ixgmq[i]->str_stats.precision(20);
    ixgmq[i]->str_stats << "INSTRUCTIONS=" << ins
			<< " CYCLES=" << cycs
			<< " IPC=" << (float)ins/cycs
			<< " LLC_MISSES=" << llc
			<< " TIME=" << ttime
			<< " WATTS=" << tnrg/ttime
			<< " AVG_ITR_PER_CORE=" << (float)nints/static_cast<uint32_t>(Cpu::Count())
			<< " ITR=" << ixgmq[i]->itr_val * 2
			<< " RAPL=" << ixgmq[i]->rapl_val;
      
    
    ebbrt::kprintf_force("INSTRUCTIONS=%llu\n", ins);
    ebbrt::kprintf_force("CYCLES=%llu\n", cycs);
    ebbrt::kprintf_force("IPC=%.2f\n", (float)ins/cycs);
    ebbrt::kprintf_force("LLC_MISSES=%llu\n", llc);
    
    ebbrt::kprintf_force("AVG_ITR_PER_CORE=%.2f\n", (float)nints/static_cast<uint32_t>(Cpu::Count()));
    ebbrt::kprintf_force("\n");*/
    ebbrt::kprintf_force("\n%15s %15s %15s\n", "num_desc","tx","rx");
    for(int j=0;j<40;j++) {
      uint32_t sumt = 0;
      uint32_t sumr = 0;
      for(uint32_t c = 0; c < static_cast<uint32_t>(Cpu::Count()); c++) {
	sumt += ixgmq[c]->tx_desc_counts[j];
	sumr += ixgmq[c]->rx_desc_counts[j];
      }      
      ebbrt::kprintf_force("%15d %15d %15d\n", j, sumt, sumr);
    }
    //ebbrt::kprintf_force("nrg=%.2lf J\n", tnrg);
    //ebbrt::kprintf_force("ttime=%.2f s time1=%.2f s time2=%.2f s\n", ttime, ixgmq[0]->totalTime, ixgmq[1]->totalTime);
    
  } else if(s == "start_idle") {
    ixgmq[i]->time_send = 0;
    ixgmq[i]->time_idle_min = 999999;
    ixgmq[i]->time_idle_max = 0;
    ixgmq[i]->total_idle_time = 0;
    //ixgmq[i]->stat_num_recv = 0;
    ixgmq[i]->idle_times_.clear();
    
  } else if(s == "stop_idle") {
    //ebbrt::kprintf_force("Core %u: idle_min=%llu idle_max=%llu stat_num_recv=%llu avg_idle=%.2lf\n", i, ixgmq[i]->time_idle_min, ixgmq[i]->time_idle_max, ixgmq[i]->stat_num_recv, (double)ixgmq[i]->total_idle_time/ixgmq[i]->stat_num_recv);
    /*for(const auto& n : ixgmq[i]->idle_times_) {
      ebbrt::kprintf_force("%u: %u\n", n.first, n.second);
      }*/
  }
  else if(s == "start_stats") {
    ebbrt::kprintf_force("start_stats on core %u\n", v);
    ixgmq[v]->collect_stats = true;
    ixgmq[v]->perfCycles.Start();
    ixgmq[v]->perfInst.Start();
    ixgmq[v]->perfLLC_miss.Start();    
    ixgmq[v]->powerMeter.Start();
  }
  else if(s == "stop_stats") {
    ebbrt::kprintf_force("stop_stats on core %u\n", v);
    ixgmq[v]->collect_stats = false;
    
    ixgmq[v]->perfCycles.Stop();
    ixgmq[v]->perfInst.Stop();
    ixgmq[v]->perfLLC_miss.Stop();    
    ixgmq[v]->powerMeter.Stop();
    
    ixgmq[v]->perfCycles.Clear();
    ixgmq[v]->perfInst.Clear();
    ixgmq[v]->perfLLC_miss.Clear();    
    ixgmq[v]->powerMeter.Clear();
  }
  else if(s == "print_stats") {
    struct IxgbeLog *il;
    union IxgbeLogEntry *ile;
    uint64_t rxb = 0, txb = 0;

    ebbrt::kprintf_force("++++++++++ PRINT_STATS START +++++++++++\n");
    il= &ixgbe_logs[v];
    ebbrt::kprintf_force("i rx_desc rx_bytes tx_desc tx_bytes instructions cycles llc_miss joules timestamp\n");
    for (int i = 0; i < il->itr_cnt; i++) {
      ile = &il->log[i];
      rxb += ile->Fields.rx_bytes;
      txb += ile->Fields.tx_bytes;
      
      ebbrt::kprintf_force("%d %d %d %d %d %llu %llu %llu %llu %llu\n",
			   i,
			   ile->Fields.rx_desc, ile->Fields.rx_bytes,
			   ile->Fields.tx_desc, ile->Fields.tx_bytes,
			   ile->Fields.ninstructions,
			   ile->Fields.ncycles,
			   ile->Fields.nllc_miss,		   
			   ile->Fields.joules,		   
			   ile->Fields.tsc);
      
    }
    ebbrt::kprintf_force("Core=%u itr_cnt=%lu total_rx_bytes=%lu rxb=%lu total_tx_bytes=%lu txb=%lu\n", v, il->itr_cnt, ixgmq[v]->total_rx_bytes, rxb, ixgmq[v]->total_tx_bytes, txb);
    ebbrt::kprintf_force("++++++++++ PRINT_STATS END +++++++++++\n");
    
    /*for(auto a : ixgmq[v]->itr_stats) {
      rxb += a.rx_bytes;
      txb += a.tx_bytes;
    }
    
    ebbrt::kprintf_force("Core=%u itr_cnt=%lu total_rx_bytes=%lu rxb=%lu total_tx_bytes=%lu txb=%lu\n", v, ixgmq[v]->itr_stats.size(), ixgmq[v]->total_rx_bytes, rxb, ixgmq[v]->total_tx_bytes, txb);
    ebbrt::kprintf_force("i rx_desc rx_bytes tx_desc tx_bytes instructions cycles llc_miss joules timestamp\n");

    for(auto a : ixgmq[v]->itr_stats) {
    ebbrt::kprintf_force("%u %u %u %u %u %llu %llu %llu %llu %llu\n",
			   c,
			   a.rx_desc,
			   a.rx_bytes,
			   a.tx_desc,
			   a.tx_bytes,
			   a.ninstructions,
			   a.ncycles,
			   a.nllc_miss,
			   a.joules,
			   a.itr_time_ns);
      c ++;
      }    */
  } else if(s == "clear_stats") {
    //ixgmq[v]->itr_stats.clear();
    memset(ixgbe_logs, 0, sizeof(ixgbe_logs));
    ixgmq[v]->total_rx_bytes = 0;
    ixgmq[v]->total_tx_bytes = 0;
  } else {
    ebbrt::kprintf_force("%s Unknown command:\n", __PRETTY_FUNCTION__);    
  }
}

void ebbrt::IxgbeDriver::Send(std::unique_ptr<IOBuf> buf, PacketInfo pinfo) {
  ebb_->Send(std::move(buf), std::move(pinfo));
}

// After packet transmission, need to mark bit in
// tx queue so that it can be used again
// TX_HEAD_WB does it automatically
void ebbrt::IxgbeDriverRep::ReclaimTx() {
  // with TX head writeback, shouldn't need to poll anymore (right?)
#ifndef TX_HEAD_WB
  tdesc_advance_tx_wbf_t* awbfx;
  uint32_t free_desc;
  
  if(ixgmq_.tx_tail_ > ixgmq_.tx_head_) {
    free_desc = IxgbeDriver::NTXDESCS - (ixgmq_.tx_tail_ - ixgmq_.tx_head_);
  } else if(ixgmq_.tx_tail_ < ixgmq_.tx_head_) {
    free_desc = IxgbeDriver::NTXDESCS - ((ixgmq_.tx_tail_+IxgbeDriver::NTXDESCS) - ixgmq_.tx_head_);
  } else {
    free_desc = IxgbeDriver::NTXDESCS;
  }
  
  // (IxgbeDriver::NTXDESCS - 1): 340 W, 1599820.2, eax=0x60
  //if(free_desc < (IxgbeDriver::NTXDESCS - 1)) {
  if(free_desc < (IxgbeDriver::NTXDESCS - 456)) {
    auto head = ixgmq_.tx_head_;
    auto tail = ixgmq_.tx_tail_;

    while(head != tail) {
      if(ixgmq_.tx_iseop[head] == true) {
	awbfx = reinterpret_cast<tdesc_advance_tx_wbf_t*>(&(ixgmq_.tx_ring_[head]));
	while(awbfx->dd == 0) {
	  asm volatile("lfence":::"memory");
	}
	ixgmq_.tx_iseop[head] = false;
      }
      head = (head + 1) % ixgmq_.tx_size_;
    }
    ixgmq_.tx_head_ = head;
  }
#endif
}

uint32_t ebbrt::IxgbeDriverRep::SendUdp(std::unique_ptr<IOBuf> buf, uint64_t len, PacketInfo pinfo) {
  uint64_t data;
  std::unique_ptr<MutUniqueIOBuf> b;
  tdesc_advance_tx_rf_t* arfx;
  uint32_t end;
  uint32_t nchains=0;
    
  // coalesce into single packet if no checksum
  //ebbrt::kprintf_force("SendUdp len=%d\n", len);
  if(buf->IsChained()) {
    b = MakeUniqueIOBuf(len);
    auto mdata = b->MutData();
    for (auto& buf_it : *buf) {
      memcpy(mdata, buf_it.Data(), buf_it.Length());
      mdata += buf_it.Length();
    }
    data = reinterpret_cast<uint64_t>(b->MutData());
  } else {
    data = reinterpret_cast<uint64_t>(buf->Data());
  }
    
  arfx = reinterpret_cast<tdesc_advance_tx_rf_t*>(&(ixgmq_.tx_ring_[ixgmq_.tx_tail_]));
  arfx->raw[0] = 0x0;
  arfx->raw[1] = 0x0;
    
  arfx->address = data;

  // Holds length in bytes of data buffer at the address pointed to by this specific descriptor.
  // Max length is 15.5 KB
  arfx->dtalen = len;
    
  // In a single-send packet, PAYLEN defines the entire packet size fetched from host memory.
  arfx->paylen = len;
    
  // crc checksum
  arfx->ifcs = 1;
    
  // rs bit should only be set when eop is set
  arfx->eop = 1;
  arfx->rs = 1;
    
  // type is advanced
  arfx->dtyp = 0x3;
  arfx->dext = 1;

  //ebbrt::kprintf("Send mcore=%u tail=%u tx_adv_rd_desc = 0x%llX 0x%X 0x%X\n", mcore, ixgmq_.tx_tail_, arfx->raw[0], (uint32_t)(arfx->raw[1] & 0xFFFFFFFF), (uint32_t)((arfx->raw[1] >> 32) & 0xFFFFFFFF));
  end = static_cast<uint32_t>(ixgmq_.tx_tail_);
  ixgmq_.tx_iseop[end] = true;
  ixgmq_.tx_tail_ = (ixgmq_.tx_tail_ + 1) % ixgmq_.tx_size_;
  nchains ++;

  return nchains;
}

uint32_t ebbrt::IxgbeDriverRep::SendTCPUnchained(std::unique_ptr<IOBuf> buf, uint64_t len, PacketInfo pinfo) {
  uint64_t data, tsodata, tsolen;
  std::unique_ptr<MutUniqueIOBuf> b;
  tdesc_advance_tx_rf_t* arfx;
  tdesc_advance_ctxt_wb_t* actx;  
  uint32_t end;
  uint32_t nchains = 0;
  
  data = reinterpret_cast<uint64_t>(buf->Data());
  //ebbrt::kprintf_force("SendTCPUnchained len=%llu\n", len);
      
  if(len > IXGBE_MAX_DATA_PER_TXD) {
    /*** CONTEXT START ***/
    actx = reinterpret_cast<tdesc_advance_ctxt_wb_t*>(&(ixgmq_.tx_ring_[ixgmq_.tx_tail_]));
    actx->raw_1 = 0x0;
    actx->raw_2 = 0x0;
    actx->iplen = IPHDR_LEN;
    actx->maclen = ETHHDR_LEN;
    // ip packet type = ipv4: 01
    actx->ipv4 = 1;
      
    if (pinfo.csum_offset == 6) {
      // l4type = udp: 00
      actx->l4t = 0;
    } else if (pinfo.csum_offset == 16) {
      // l4type = tcp: 01
      actx->l4t = 1;
    } 
      
    // for context descriptor 0x2
    actx->dtyp = 0x2;
    // descriptor extension, one for advanced mode
    actx->dext = 1;
    // from Linux??, ignored when no TSE
    actx->mss = 1448;
    // TCP header length, with no tcp options == 20, ignored when no TSE
    actx->l4len = pinfo.tcp_hdr_len;
    ixgmq_.tx_tail_ = (ixgmq_.tx_tail_ + 1) % ixgmq_.tx_size_;
    nchains++;
    /*** CONTEXT END ***/
      
    //first descriptor           
    arfx = reinterpret_cast<tdesc_advance_tx_rf_t*>(&(ixgmq_.tx_ring_[ixgmq_.tx_tail_]));
    arfx->raw[0] = 0x0;
    arfx->raw[1] = 0x0;
    arfx->address = data;
    // Holds length in bytes of data buffer at the address pointed to by this specific descriptor.
    // Max length is 15.5 KB
    arfx->dtalen = IXGBE_MAX_DATA_PER_TXD;
    arfx->dtyp = 0x3;
    arfx->ifcs = 1;
    arfx->dext = 1;
    arfx->tse = 1;
    arfx->ixsm = 1;
    arfx->txsm = 1;
    // In Tcp Segmentation Mode (TSE), PAYLEN defines the TCP/UDP payload length, so no header length
    arfx->paylen = pinfo.tcp_len;
    ixgmq_.tx_tail_ = (ixgmq_.tx_tail_ + 1) % ixgmq_.tx_size_;
    nchains++;
    
    tsodata = data;
    tsolen = len;
      
    while(tsolen > IXGBE_MAX_DATA_PER_TXD) {
      tsodata += IXGBE_MAX_DATA_PER_TXD;
      tsolen -= IXGBE_MAX_DATA_PER_TXD;

      arfx = reinterpret_cast<tdesc_advance_tx_rf_t*>(&(ixgmq_.tx_ring_[ixgmq_.tx_tail_]));
      arfx->raw[0] = 0x0;
      arfx->raw[1] = 0x0;
      arfx->dtyp = 0x3;
      arfx->dext = 1;
      arfx->tse = 1;
      arfx->ifcs = 1;
      arfx->address = tsodata;
	
      if(tsolen > IXGBE_MAX_DATA_PER_TXD) {
	arfx->dtalen = IXGBE_MAX_DATA_PER_TXD;
	ixgmq_.tx_tail_ = (ixgmq_.tx_tail_ + 1) % ixgmq_.tx_size_;
	nchains++;
      } else { 
	// last descriptor
	arfx->dtalen = tsolen;
	arfx->eop = 1;
	arfx->rs = 1;
	    
	end = ixgmq_.tx_tail_;
	ixgmq_.tx_iseop[end] = true;
	ixgmq_.tx_tail_ = (ixgmq_.tx_tail_ + 1) % ixgmq_.tx_size_;
	nchains++;
      }
    }      
  }
  else if(len > 1490 && len < IXGBE_MAX_DATA_PER_TXD) {
      
    /*** CONTEXT START ***/
    actx = reinterpret_cast<tdesc_advance_ctxt_wb_t*>(&(ixgmq_.tx_ring_[ixgmq_.tx_tail_]));
    actx->raw_1 = 0x0;
    actx->raw_2 = 0x0;
    actx->iplen = IPHDR_LEN;
    actx->maclen = ETHHDR_LEN;
    // ip packet type = ipv4: 01
    actx->ipv4 = 1;
      
    if (pinfo.csum_offset == 6) {
      // l4type = udp: 00
      actx->l4t = 0;
    } else if (pinfo.csum_offset == 16) {
      // l4type = tcp: 01
      actx->l4t = 1;
    } 
      
    // for context descriptor 0b0010
    actx->dtyp = 0x2;
    // descriptor extension, one for advanced mode
    actx->dext = 1;
    // from Linux??, ignored when no TSE
    actx->mss = 1448;
    // TCP header length, with no tcp options == 20, ignored when no TSE
    actx->l4len = pinfo.tcp_hdr_len;
    ixgmq_.tx_tail_ = (ixgmq_.tx_tail_ + 1) % ixgmq_.tx_size_;
    nchains++;
    /*** CONTEXT END ***/
      
    arfx = reinterpret_cast<tdesc_advance_tx_rf_t*>(&(ixgmq_.tx_ring_[ixgmq_.tx_tail_]));
    arfx->raw[0] = 0x0;
    arfx->raw[1] = 0x0;
    arfx->address = data;
      
    // Holds length in bytes of data buffer at the address pointed to by this specific descriptor.
    // Max length is 15.5 KB
    arfx->dtalen = len;
    arfx->dtyp = 0x3;
    arfx->eop = 1;
    arfx->rs = 1;
    arfx->ifcs = 1;
    arfx->dext = 1;
    arfx->tse = 1;
      
    arfx->ixsm = 1;
    arfx->txsm = 1;
    // In Tcp Segmentation Mode (TSE), PAYLEN defines the TCP/UDP payload size
    arfx->paylen = pinfo.tcp_len;
    end = ixgmq_.tx_tail_;
    ixgmq_.tx_iseop[end] = true;
    ixgmq_.tx_tail_ = (ixgmq_.tx_tail_ + 1) % ixgmq_.tx_size_;
    nchains++;
  } else { /**** NOT IN TSE mode****/
    /*** CONTEXT START ***/
    actx = reinterpret_cast<tdesc_advance_ctxt_wb_t*>(&(ixgmq_.tx_ring_[ixgmq_.tx_tail_]));
    actx->raw_1 = 0x0;
    actx->raw_2 = 0x0;
    actx->iplen = IPHDR_LEN;
    actx->maclen = ETHHDR_LEN;
    // ip packet type = ipv4: 01
    actx->ipv4 = 1;
      
    if (pinfo.csum_offset == 6) {
      // l4type = udp: 00
      actx->l4t = 0;
    } else if (pinfo.csum_offset == 16) {
      // l4type = tcp: 01
      actx->l4t = 1;
    } 
      
    // for context descriptor 0b0010
    actx->dtyp = 0x2;
    // descriptor extension, one for advanced mode
    actx->dext = 1;
    // from Linux, ignored when no TSE
    actx->mss = 0;
    // TCP header length, with no tcp options == 20, ignored when no TSE
    actx->l4len = 0;    
    ixgmq_.tx_tail_ = (ixgmq_.tx_tail_ + 1) % ixgmq_.tx_size_;
    nchains++;
    /*** CONTEXT END ***/
      
    arfx = reinterpret_cast<tdesc_advance_tx_rf_t*>(&(ixgmq_.tx_ring_[ixgmq_.tx_tail_]));
    arfx->raw[0] = 0x0;
    arfx->raw[1] = 0x0;
    arfx->address = data;
      
    // Holds length in bytes of data buffer at the address pointed to by this specific descriptor.
    // Max length is 15.5 KB
    arfx->dtalen = len;
    arfx->paylen = len;
      
    arfx->dtyp = 0x3;
    arfx->eop = 1;
    arfx->rs = 1;
    arfx->ifcs = 1;

    arfx->dext = 1;
    arfx->tse = 0;
      
    arfx->ixsm = 1;
    arfx->txsm = 1;
    end = ixgmq_.tx_tail_;
    ixgmq_.tx_iseop[end] = true;
    ixgmq_.tx_tail_ = (ixgmq_.tx_tail_ + 1) % ixgmq_.tx_size_;
    nchains++;
  }

  return nchains;
}

uint32_t ebbrt::IxgbeDriverRep::SendTCPChained(std::unique_ptr<IOBuf> buf, uint64_t len, uint64_t num_chains, PacketInfo pinfo) {
  uint64_t data, i;
  std::unique_ptr<MutUniqueIOBuf> b;
  tdesc_advance_tx_rf_t* arfx;
  tdesc_advance_ctxt_wb_t* actx;  
  uint32_t end;
  uint32_t nchains=0;
  //uint32_t mcore = static_cast<uint32_t>(Cpu::GetMine());
  
  //ebbrt::kprintf_force("** SendTCPChained num_chains=%llu len=%llu START ** \n", num_chains, len);
  if(len <= 1490) {
    /*** CONTEXT START ***/
    actx = reinterpret_cast<tdesc_advance_ctxt_wb_t*>(&(ixgmq_.tx_ring_[ixgmq_.tx_tail_]));
    actx->raw_1 = 0x0;
    actx->raw_2 = 0x0;
    actx->iplen = IPHDR_LEN;
    actx->maclen = ETHHDR_LEN;
    // ip packet type = ipv4: 01
    actx->ipv4 = 1;
      
    if (pinfo.csum_offset == 6) {
      // l4type = udp: 00
      actx->l4t = 0;
    } else if (pinfo.csum_offset == 16) {
      // l4type = tcp: 01
      actx->l4t = 1;
    } 
      
    // for context descriptor 0b0010
    actx->dtyp = 0x2;
    // descriptor extension, one for advanced mode
    actx->dext = 1;
    // from Linux, ignored when no TSE
    actx->mss = 0;
    // TCP header length, with no tcp options == 20, ignored when no TSE
    actx->l4len = 0;    
    //ebbrt::kprintf("Send mcore=%u tail=%u tx_adv_ctxt_desc = 0x%llX 0x%X 0x%X\n", mcore, ixgmq_.tx_tail_, actx->raw_1, (uint32_t)(actx->raw_2 & 0xFFFFFFFF), (uint32_t)((actx->raw_2 >> 32) & 0xFFFFFFFF));      
    ixgmq_.tx_tail_ = (ixgmq_.tx_tail_ + 1) % ixgmq_.tx_size_;
    nchains ++;
    /*** CONTEXT END ***/

    i = 0;
    for (auto& buf_it : *buf) {	  
      data = reinterpret_cast<uint64_t>(buf_it.Data());
	  
      arfx = reinterpret_cast<tdesc_advance_tx_rf_t*>(&(ixgmq_.tx_ring_[ixgmq_.tx_tail_]));
      arfx->raw[0] = 0x0;
      arfx->raw[1] = 0x0;
      arfx->address = data;
      
      // Holds length in bytes of data buffer at the address pointed to by this specific descriptor.
      // Max length is 15.5 KB
      arfx->dtalen = buf_it.Length();
      // only valid on first descriptor of packet
      if(i == 0) {
	arfx->paylen = len;
      }
	  
      arfx->rs = 1;
      arfx->dtyp = 0x3;
      //only valid on last descriptor making up packet
      if(i == (num_chains-1)) {
	arfx->eop = 1;
      }
	  
      arfx->ifcs = 1;

      arfx->dext = 1;
      arfx->tse = 0;
      
      arfx->ixsm = 1;
      
      // if need TCP checksum offload
      arfx->txsm = 1;
      end = ixgmq_.tx_tail_;
      ixgmq_.tx_iseop[end] = true;
      ixgmq_.tx_tail_ = (ixgmq_.tx_tail_ + 1) % ixgmq_.tx_size_;
      nchains ++;
      i++;
    }
  } else {
    /*** CONTEXT START ***/
    actx = reinterpret_cast<tdesc_advance_ctxt_wb_t*>(&(ixgmq_.tx_ring_[ixgmq_.tx_tail_]));
    actx->raw_1 = 0x0;
    actx->raw_2 = 0x0;
    actx->iplen = IPHDR_LEN;
    actx->maclen = ETHHDR_LEN;
    // ip packet type = ipv4: 01
    actx->ipv4 = 1;
      
    if (pinfo.csum_offset == 6) {
      // l4type = udp: 00
      actx->l4t = 0;
    } else if (pinfo.csum_offset == 16) {
      // l4type = tcp: 01
      actx->l4t = 1;
    } 
      
    // for context descriptor 0x2
    actx->dtyp = 0x2;
    // descriptor extension, one for advanced mode
    actx->dext = 1;
    // from Linux??, ignored when no TSE
    actx->mss = 1448;
    // TCP header length, with no tcp options == 20, ignored when no TSE
    actx->l4len = pinfo.tcp_hdr_len;
    ixgmq_.tx_tail_ = (ixgmq_.tx_tail_ + 1) % ixgmq_.tx_size_;
    nchains ++;
    /*** CONTEXT END ***/

    i = 0;
    for (auto& buf_it : *buf) {
      if (buf_it.Length() > IXGBE_MAX_DATA_PER_TXD) {

	//ebbrt::kprintf_force("*** buf_it.Length():%u > IXGBE_MAX_DATA_PER_TXD:%u\n", buf_it.Length(), IXGBE_MAX_DATA_PER_TXD);
	
	data = reinterpret_cast<uint64_t>(buf_it.Data());
	uint64_t tmp_len = buf_it.Length();
	uint64_t offset = 0;
		
	while(tmp_len > IXGBE_MAX_DATA_PER_TXD) {
	  //ebbrt::kprintf_force("*** i=%llu data=0x%llX tmp_len=%llu\n", i, data+offset, tmp_len);
	  
	  arfx = reinterpret_cast<tdesc_advance_tx_rf_t*>(&(ixgmq_.tx_ring_[ixgmq_.tx_tail_]));
	  arfx->raw[0] = 0x0;
	  arfx->raw[1] = 0x0;
	  arfx->address = data + offset;
	  // Holds length in bytes of data buffer at the address pointed to by this specific descriptor.
	  // Max length is 15.5 KB
	  arfx->dtalen = IXGBE_MAX_DATA_PER_TXD;
	  offset += IXGBE_MAX_DATA_PER_TXD;
	  tmp_len -= IXGBE_MAX_DATA_PER_TXD;
	  arfx->dtyp = 0x3;
	  arfx->ifcs = 1;
	  arfx->dext = 1;
	  arfx->tse = 1;
      
	  // first descriptor
	  if (i == 0) {
	    arfx->ixsm = 1;
	    arfx->txsm = 1;
	    
	    // In Tcp Segmentation Mode (TSE), PAYLEN defines the TCP/UDP payload length, so no header length
	    // only valid on first descriptor
	    arfx->paylen = pinfo.tcp_len;	    
	  } 
	  
	  i ++;
	  ixgmq_.tx_tail_ = (ixgmq_.tx_tail_ + 1) % ixgmq_.tx_size_;
	  nchains ++;
	}

	if(tmp_len) {
	  //ebbrt::kprintf_force("*** i=%llu data=0x%llX tmp_len=%llu\n", i, data+offset, tmp_len);
	  
	  arfx = reinterpret_cast<tdesc_advance_tx_rf_t*>(&(ixgmq_.tx_ring_[ixgmq_.tx_tail_]));
	  arfx->raw[0] = 0x0;
	  arfx->raw[1] = 0x0;
	  arfx->address = data + offset;
	  // Holds length in bytes of data buffer at the address pointed to by this specific descriptor.
	  // Max length is 15.5 KB
	  arfx->dtalen = tmp_len;
	  offset += tmp_len;
	  tmp_len -= tmp_len;
	  arfx->dtyp = 0x3;
	  arfx->ifcs = 1;
	  arfx->dext = 1;
	  arfx->tse = 1;
	  
	  i ++;
	  ixgmq_.tx_tail_ = (ixgmq_.tx_tail_ + 1) % ixgmq_.tx_size_;
	  nchains ++;
	}
	
      } else { // non special case < IXGBE_MAX_DATA_PER_TXD
	data = reinterpret_cast<uint64_t>(buf_it.Data());

	arfx = reinterpret_cast<tdesc_advance_tx_rf_t*>(&(ixgmq_.tx_ring_[ixgmq_.tx_tail_]));
	arfx->raw[0] = 0x0;
	arfx->raw[1] = 0x0;
	arfx->address = data;
	// Holds length in bytes of data buffer at the address pointed to by this specific descriptor.
	// Max length is 15.5 KB
	arfx->dtalen = buf_it.Length(); //GBE_MAX_DATA_PER_TXD;
	arfx->dtyp = 0x3;
	arfx->ifcs = 1;
	arfx->dext = 1;
	arfx->tse = 1;
      
	//ebbrt::kprintf_force("\t SendTCPChained i=%u len=%llu\n", i, buf_it.Length());
	
	// first descriptor
	if (i == 0) {
	  arfx->ixsm = 1;
	  arfx->txsm = 1;
	  
	  // In Tcp Segmentation Mode (TSE), PAYLEN defines the TCP/UDP payload length, so no header length
	  // only valid on first descriptor
	  arfx->paylen = pinfo.tcp_len;	    
	}
	
	i ++;
	ixgmq_.tx_tail_ = (ixgmq_.tx_tail_ + 1) % ixgmq_.tx_size_;
	nchains ++;
      }
    }

    arfx->eop = 1;
    arfx->rs = 1;

    if(i > 40) {
      ebbrt::kabort("Error i=%llu, num chains greater than 40\n", i);
    }    
    
    /*i = 0;
    for (auto& buf_it : *buf) {
      if (buf_it.Length() > IXGBE_MAX_DATA_PER_TXD) {
      ebbrt::kprintf("TSE buf_it.Length() = %u > IXGBE_MAX_DATA_PER_TXD num_chains=%llu i=%llu\n", buf_it.Length(), num_chains, i);
	return;
      }      
      
      data = reinterpret_cast<uint64_t>(buf_it.Data());

      arfx = reinterpret_cast<tdesc_advance_tx_rf_t*>(&(ixgmq_.tx_ring_[ixgmq_.tx_tail_]));
      arfx->raw[0] = 0x0;
      arfx->raw[1] = 0x0;
      arfx->address = data;
      // Holds length in bytes of data buffer at the address pointed to by this specific descriptor.
      // Max length is 15.5 KB
      arfx->dtalen = buf_it.Length(); //GBE_MAX_DATA_PER_TXD;
      arfx->dtyp = 0x3;
      arfx->ifcs = 1;
      arfx->dext = 1;
      arfx->tse = 1;

      //ebbrt::kprintf_force("\t SendTCPChained i=%u len=%llu\n", i, buf_it.Length());
      
      // first descriptor
      if (i == 0) {
	arfx->ixsm = 1;
	arfx->txsm = 1;
	  
	// In Tcp Segmentation Mode (TSE), PAYLEN defines the TCP/UDP payload length, so no header length
	// only valid on first descriptor
	arfx->paylen = pinfo.tcp_len;	    
      } else if(i == (num_chains-1)) { // last descriptor
	arfx->eop = 1;
	arfx->rs = 1;
	arfx->dtalen = buf_it.Length();
      } else {
	arfx->dtalen = buf_it.Length();
      }

      i ++;
      ixgmq_.tx_tail_ = (ixgmq_.tx_tail_ + 1) % ixgmq_.tx_size_;
      //asm volatile("sfence" ::: "memory");  
      // WriteTdt_1(mcore, ixgmq_.tx_tail_);
    }*/	
  }
  return nchains;
  //ebbrt::kprintf_force("** SendTcpChained END **\n");
}

void ebbrt::IxgbeDriverRep::Send(std::unique_ptr<IOBuf> buf, PacketInfo pinfo) {
  uint64_t len, num_chains;
  uint32_t mcore = static_cast<uint32_t>(Cpu::GetMine());
  std::unique_ptr<MutUniqueIOBuf> b;
  
  // On TSO, the maximum PAYLEN can be up to 2^18 - 1
  len = buf->ComputeChainDataLength();
  ixgmq_.stat_num_tx_bytes += len;
  ixgmq_.total_tx_bytes += len;
  if (len > 262144) {
    ebbrt::kprintf_force("\t kabort Send() len=%lld greater than TSO limit of 262144 bytes\n", len);
    return;
  }
  num_chains = buf->CountChainElements();

#ifndef TX_HEAD_WB
  ReclaimTx();
#endif
  
//#ifdef RSC_EN  
  // if no IP/TCP checksum - likely UDP packet
  if (!(pinfo.flags & PacketInfo::kNeedsCsum) && !(pinfo.flags & PacketInfo::kNeedsIpCsum)) {
    ixgmq_.stat_num_tx_desc += SendUdp(std::move(buf), len, pinfo);
  } else { // TCP Packet
    // hardware limits sending over 40 descriptors per packet, have to manually coalesce here
    // hopefully not too often
    if(num_chains > 38) {
      ixgmq_.tx_desc_counts[39] ++;
      //ebbrt::kprintf_force("*** num_chains=%d > 38\n", num_chains);
      b = MakeUniqueIOBuf(len);
      auto mdata = b->MutData();
      for (auto& buf_it : *buf) {
	memcpy(mdata, buf_it.Data(), buf_it.Length());
	mdata += buf_it.Length();
      }
      //data = reinterpret_cast<uint64_t>(b->MutData());
      
      ixgmq_.stat_num_tx_desc += SendTCPUnchained(std::move(b), len, pinfo);
      
    } else if(num_chains > 1 && num_chains <= 39) {
      ixgmq_.tx_desc_counts[num_chains] ++;
      ixgmq_.stat_num_tx_desc += SendTCPChained(std::move(buf), len, num_chains, pinfo);
    } else { //Not Chained
      ixgmq_.tx_desc_counts[1] ++;
      ixgmq_.stat_num_tx_desc += SendTCPUnchained(std::move(buf), len, pinfo);
    }
  }
  
  
/*#else
  // if no IP/TCP checksum - likely UDP packet
  if (!(pinfo.flags & PacketInfo::kNeedsCsum) && !(pinfo.flags & PacketInfo::kNeedsIpCsum)) {
    ixgmq_.stat_num_tx_desc += SendUdp(std::move(buf), len, pinfo);
  } else { // TCP Packet
    ixgmq_.tx_desc_counts[1] ++;
    
    if(num_chains > 1) {
      b = MakeUniqueIOBuf(len);
      auto mdata = b->MutData();
      for (auto& buf_it : *buf) {
	memcpy(mdata, buf_it.Data(), buf_it.Length());
	mdata += buf_it.Length();
      }
      ixgmq_.stat_num_tx_desc += SendTCPUnchained(std::move(b), len, pinfo);
    } else {
      ixgmq_.stat_num_tx_desc += SendTCPUnchained(std::move(buf), len, pinfo);
    }
  }
  #endif*/

  //ebbrt::kprintf("\t Send() core=%u head=%u tail=%u free_desc=%u\n", mcore, ixgmq_.tx_head_, ixgmq_.tx_tail_, free_desc);
  asm volatile("sfence" ::: "memory");
  //ebbrt::kprintf("\t Send() core=%u head=%u last_tail=%u tail=%u free_desc=%u\n", mcore, ixgmq_.tx_head_, ixgmq_.tx_last_tail_, ixgmq_.tx_tail_, free_desc);
  
  WriteTdt_1(mcore, ixgmq_.tx_tail_);
  
  //while(arfx->dd == 0) {
  // makes sure all reads are finished before checking again
  //asm volatile("lfence":::"memory");
  //}
  
  //auto d = ebbrt::clock::Wall::Now().time_since_epoch();
  //ixgmq_.time_send = std::chrono::duration_cast<std::chrono::microseconds>(d).count();
  
}

void ebbrt::IxgbeDriver::WriteRxctrl(uint32_t m) {
  // Disable RXCTRL - 8.2.3.8.10
  bar0_.Write32(0x03000, m);
}

void ebbrt::IxgbeDriver::WriteDmatxctl(uint32_t m) {
  uint32_t reg;

  reg = bar0_.Read32(0x04A80);
  ebbrt::kprintf("0x04A80: DMATXCTL 0x%08X - reset to 0x%08X\n", reg, reg & m);

  // DMATXCTL - 8.2.3.9.2
  bar0_.Write32(0x04A80, reg & m);
}
void ebbrt::IxgbeDriver::WriteDmatxctl_te(uint32_t m) {
  auto reg = bar0_.Read32(0x04A80);
  ebbrt::kprintf("DMATXCTL= 0x%X\n", reg | m);
  bar0_.Write32(0x04A80, reg | m);
}

//8.2.3.5.10 Extended Interrupt Auto Mask Enable registers — EIAM[n] (0x00AD0 + 4*(n-1), n=1...2; RW)
void ebbrt::IxgbeDriver::WriteEiam(uint32_t n, uint32_t m) {
  bar0_.Write32(0x00AD0 + 4*n, m);
}
  
// 8.2.3.5.18 - General Purpose Interrupt Enable — GPIE (0x00898; RW)
void ebbrt::IxgbeDriver::WriteGpie(uint32_t m) {
  //auto reg = bar0_.Read32(0x00898);
  //bar0_.Write32(0x00898, reg | m);
  bar0_.Write32(0x00898, m);
}

// 8.2.3.5.1 Extended Interrupt Cause Register- EICR (0x00800; RW1C)
void ebbrt::IxgbeDriver::ReadEicr() {
  /* Note
   * The EICR is also cleared on read if GPIE.OCD bit is cleared. When the
   * GPIE.OCD bit is set, then only bits 16...29 are cleared on read.
   */
  // 8.2.3.5.18 General Purpose Interrupt Enable — GPIE (0x00898;RW)
  uint32_t reg;
  reg = bar0_.Read32(0x00898);
  ebbrt::kbugon((reg & 0x20), "GPIE.OCD not cleared\n");

  reg = bar0_.Read32(0x00800);
  ebbrt::kprintf("First Read - 0x00800: EICR 0x%08X, ", reg);

  reg = bar0_.Read32(0x00800);
  ebbrt::kprintf("Second Read - EICR 0x%08X\n", reg);
}
void ebbrt::IxgbeDriver::WriteEicr(uint32_t m) {
  auto reg = bar0_.Read32(0x00800);
  bar0_.Write32(0x00800, reg | m);
}

// 8.2.3.5.3 Extended Interrupt Mask Set/Read Register- EIMS (0x00880; RWS)
uint32_t ebbrt::IxgbeDriver::ReadEims() { return bar0_.Read32(0x00880); }
void ebbrt::IxgbeDriver::WriteEims(uint32_t m) { bar0_.Write32(0x00880, m); }

// 8.2.3.5.4 Extended Interrupt Mask Clear Register- EIMC (0x00888; WO)
void ebbrt::IxgbeDriver::WriteEimc(uint32_t m) { bar0_.Write32(0x00888, m); }

// 8.2.3.5.5 Extended Interrupt Auto Clear Register — EIAC (0x00810; RW)
void ebbrt::IxgbeDriver::WriteEiac(uint32_t m) {
  auto reg = bar0_.Read32(0x00810);
  bar0_.Write32(0x00810, reg | m);
}

// 8.2.3.5.8 Extended Interrupt Mask Set/Read Registers — EIMS[n] (0x00AA0 +
// 4*(n-1), n=1...2; RWS)
void ebbrt::IxgbeDriver::WriteEimsn(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x00AA0 + 4 * n);
  bar0_.Write32(0x00AA0 + 4 * n, reg | m);
}

// 8.2.3.5.12
// Extended Interrupt Throttle Registers — EITR[n]
// (0x00820 + 4*n, n=0...23 and 0x012300 + 4*(n-24),
// n=24...128; RW)
void ebbrt::IxgbeDriver::WriteEitr(uint32_t n, uint32_t m) {
  ebbrt::kbugon(n > 128, "%s error\n", __FUNCTION__);

  if (n < 24) {
    bar0_.Write32(0x00820 + 4 * n, m);
  } else {
    bar0_.Write32(0x012300 + 4 * (n - 24), m);
  }
}

// 8.2.3.9.10 Transmit Descriptor Control — TXDCTL[n] (0x06028+0x40*n,
// n=0...127; RW)
void ebbrt::IxgbeDriver::WriteTxdctl(uint32_t n, uint32_t m) {
  bar0_.Write32(0x06028 + (0x40 * n), m);
}
uint8_t ebbrt::IxgbeDriver::ReadTxdctl_enable(uint32_t n) {
  auto reg = bar0_.Read32(0x06028 + 0x40 * n);
  return (reg >> 25) & 0x1;
}

// 8.2.3.8.6 Receive Descriptor Control — RXDCTL[n] (0x01028 +
// 0x40*n, n=0...63 and 0x0D028 + 0x40*(n-64), n=64...127; RW)
void ebbrt::IxgbeDriver::WriteRxdctl_1(uint32_t n, uint32_t m) {
  bar0_.Write32(0x01028 + (0x40 * n), m);
}
void ebbrt::IxgbeDriver::WriteRxdctl_1_enable(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x01028 + (0x40 * n));
  bar0_.Write32(0x01028 + (0x40 * n), reg | m);
}

uint8_t ebbrt::IxgbeDriver::ReadRxdctl_1_enable(uint32_t n) {
  auto reg = bar0_.Read32(0x01028 + (0x40 * n));
  return (reg >> 25) & 0x1;
}

void ebbrt::IxgbeDriver::WriteRxdctl_2(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0D028 + (0x40 * n), m);
}

// 8.2.3.27.14 PF VM L2 Control Register — PFVML2FLT[n] (0x0F000 + 4*n,
// n=0...63; RW)
void ebbrt::IxgbeDriver::WritePfvml2flt(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0F000 + 4 * n, m);
}

// 8.2.3.9.14 Manageability Transmit TC Mapping — MNGTXMAP (0x0CD10; RW)
void ebbrt::IxgbeDriver::WriteMngtxmap(uint32_t m) {
  bar0_.Write32(0x0CD10, m);
}

// 8.2.3.1.1 Device Control Register — CTRL (0x00000 / 0x00004;RW)
void ebbrt::IxgbeDriver::WriteCtrl(uint32_t m) { bar0_.Write32(0x0, m); }
void ebbrt::IxgbeDriver::ReadCtrl() {
  uint32_t reg;
  reg = bar0_.Read32(0x0);
  ebbrt::kprintf("%s = 0x%X\n", __FUNCTION__, reg);
}

// 8.2.3.1.3 Extended Device Control Register — CTRL_EXT (0x00018; RW)
void ebbrt::IxgbeDriver::WriteCtrlExt(uint32_t m) {
  //auto reg = bar0_.Read32(0x00018);
  //bar0_.Write32(0x00018, reg | m);
  bar0_.Write32(0x00018, m);
}

// 8.2.3.7.1 Filter Control Register — FCTRL (0x05080; RW)
void ebbrt::IxgbeDriver::WriteFctrl(uint32_t m) { bar0_.Write32(0x05080, m); }

// 8.2.3.24.9 Flexible Host Filter Table Registers — FHFT (0x09000 — 0x093FC and
// 0x09800 — 0x099FC; RW)
void ebbrt::IxgbeDriver::WriteFhft_1(uint32_t n, uint32_t m) {
  bar0_.Write32(0x09000, m);
}
void ebbrt::IxgbeDriver::WriteFhft_2(uint32_t n, uint32_t m) {
  bar0_.Write32(0x09800, m);
}

// 8.2.3.1.2 Device Status Register — STATUS (0x00008; RO)
bool ebbrt::IxgbeDriver::ReadStatusPcieMes() {
  auto reg = bar0_.Read32(0x8);
  return !(reg & 0x80000);
}
uint8_t ebbrt::IxgbeDriver::ReadStatusLanId() {
  auto reg = bar0_.Read32(0x8);
  return (reg >> 2) & 0x3;
}

// 8.2.3.3.2 Flow Control Transmit Timer Value n — FCTTVn (0x03200 + 4*n,
// n=0...3; RW)
void ebbrt::IxgbeDriver::WriteFcttv(uint32_t n, uint32_t m) {
  bar0_.Write32(0x03200 + (4 * n), m);
}

// 8.2.3.3.3 Flow Control Receive Threshold Low — FCRTL[n] (0x03220 + 4*n,
// n=0...7; RW)
void ebbrt::IxgbeDriver::WriteFcrtl(uint32_t n, uint32_t m) {
  bar0_.Write32(0x03220 + (4 * n), m);
}

// 8.2.3.3.4 Flow Control Receive Threshold High — FCRTH[n] (0x03260 + 4*n,
// n=0...7; RW)
void ebbrt::IxgbeDriver::WriteFcrth(uint32_t n, uint32_t m) {
  bar0_.Write32(0x03260 + (4 * n), m);
}

// 8.2.3.3.5 Flow Control Refresh Threshold Value — FCRTV (0x032A0; RW)
void ebbrt::IxgbeDriver::WriteFcrtv(uint32_t m) { bar0_.Write32(0x032A0, m); }

// 8.2.3.3.7 Flow Control Configuration — FCCFG (0x03D00; RW)
void ebbrt::IxgbeDriver::WriteFccfg(uint32_t m) { bar0_.Write32(0x03D00, m); }

// 8.2.3.2.2 EEPROM Read Register — EERD (0x10014; RW)
void ebbrt::IxgbeDriver::WriteEerd(uint32_t m) { bar0_.Write32(0x10014, m); }
bool ebbrt::IxgbeDriver::ReadEerdDone() {
  auto reg = bar0_.Read32(0x10014);
  return !!(reg & 0x2);  // return true when Read Done = 1
}

uint16_t ebbrt::IxgbeDriver::ReadEerdData() {
  auto reg = bar0_.Read32(0x10014);
  return (reg >> 16) & 0xFFFF;
}

uint16_t ebbrt::IxgbeDriver::ReadEeprom(uint16_t offset) {
  WriteEerd(offset << 2 | 1);
  // TODO: Timeout
  while (ReadEerdDone() == 0)
    ;
  return ReadEerdData();
}

// 8.2.3.22.32 - Core Analog Configuration Register — CoreCTL (0x014F00; RW)
void ebbrt::IxgbeDriver::WriteCorectl(uint16_t m) {
  bar0_.Write32(0x014F00, 0x0 | m);
}

// 8.2.3.22.19 Auto Negotiation Control Register — AUTOC (0x042A0; RW)
void ebbrt::IxgbeDriver::WriteAutoc(uint32_t m) {
  auto reg = bar0_.Read32(0x042A0);
  bar0_.Write32(0x042A0, reg | m);
}
uint8_t ebbrt::IxgbeDriver::ReadAutocRestartAn() {
  auto reg = bar0_.Read32(0x042A0);
  return (reg >> 12) & 0x1;
}

// 8.2.3.22.23 Auto Negotiation Link Partner Link Control Word 1 Register —
// ANLP1 (0x042B0; RO)
uint8_t ebbrt::IxgbeDriver::ReadAnlp1() {
  auto reg = bar0_.Read32(0x042B0);
  return (reg >> 16) & 0xFF;
}

// 8.2.3.2.1 EEPROM/Flash Control Register — EEC (0x10010; RW)
uint8_t ebbrt::IxgbeDriver::ReadEecAutoRd() {
  auto reg = bar0_.Read32(0x10010);
  return (reg >> 9) & 0xFF;
}

// 8.2.3.7.7 Multicast Table Array — MTA[n] (0x05200 + 4*n, n=0...127; RW)
void ebbrt::IxgbeDriver::WriteMta(uint32_t n, uint32_t m) {
  bar0_.Write32(0x05200 + (4 * n), m);
}

// 8.2.3.7.11 VLAN Filter Table Array — VFTA[n] (0x0A000 + 4*n,n=0...127; RW)
void ebbrt::IxgbeDriver::WriteVfta(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0A000 + (4 * n), m);
}

// 8.2.3.27.15 PF VM VLAN Pool Filter — PFVLVF[n] (0x0F100 + 4*n, n=0...63; RW)
void ebbrt::IxgbeDriver::WritePfvlvf(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0F100 + 4 * n, m);
}

// 8.2.3.27.16 PF VM VLAN Pool Filter Bitmap — PFVLVFB[n] (0x0F200 + 4*n,
// n=0...127; RW)
void ebbrt::IxgbeDriver::WritePfvlvfb(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0F200 + 4 * n, m);
}

// 8.2.3.7.23 Rx Filter ECC Err Insertion 0 — RXFECCERR0 (0x051B8; RW)
void ebbrt::IxgbeDriver::WriteRxfeccerr0(uint32_t m) {
  auto reg = bar0_.Read32(0x051B8);
  bar0_.Write32(0x051B8, reg | m);
}

// Checks the MAC's EEPROM to see if it supports a given SFP+ module type, if
// 1360
// so it returns the offsets to the phy init sequence block.
// also based on
// http://lxr.free-electrons.com/source/drivers/net/ethernet/intel/ixgbe/ixgbe_phy.c?v=3.14#L1395
// https://github.com/freebsd/freebsd/blob/386ddae58459341ec567604707805814a2128a57/sys/dev/ixgbe/ixgbe_82599.c#L173
void ebbrt::IxgbeDriver::PhyInit() {

  uint16_t list_offset;
  uint16_t data_offset = 0x0;
  uint16_t data_value;
  uint16_t sfp_id;
  uint16_t sfp_type = 0x4; /* SPF_DA_CORE1 */

  /* IXGBE_PHY_INIT_OFFSET_NL */
  list_offset = ReadEeprom(0x002B);

  if ((list_offset == 0x0) || (list_offset == 0xFFFF)) {
    return;
  }

  /* Shift offset to first ID word */
  list_offset++;

  sfp_id = ReadEeprom(list_offset);

  while (sfp_id != 0xFFFF) {
    if (sfp_id == sfp_type) {
      list_offset++;
      data_offset = ReadEeprom(list_offset);
      if ((data_offset == 0x0) || (data_offset == 0xFFFF)) {
        ebbrt::kprintf("sfp init failed\n");
        return;
      } else {
        break;
      }
    } else {
      list_offset += 2;
      sfp_id = ReadEeprom(list_offset);
    }
    list_offset++;
  }

  if (sfp_id == 0xFFFF) {
    ebbrt::kprintf("sfp init failed\n");
    return;
  }

  ebbrt::kprintf("data offset -> 0x%x\n", data_offset);

  SwfwLockPhy();

  data_value = ReadEeprom(++data_offset);
  while (data_value != 0xFFFF) {
    ebbrt::kprintf("data_value -> 0x%x\n", data_value);
    WriteCorectl(data_value);
    data_value = ReadEeprom(++data_offset);
  }
  SwfwUnlockPhy();

  ebbrt::clock::SleepMilli(20);

  WriteAutoc(0x0 << 13 | 0x1 << 12);
  while (ReadAnlp1() != 0)
    ;  // TODO: timeout

  WriteAutoc(0x3 << 13 | 0x1 << 12);
  while (ReadAutocRestartAn() != 0)
    ;  // TODO: timeout

  ebbrt::kprintf("PHY init done\n");
}

// 8.2.3.7.8 Receive Address Low — RAL[n] (0x0A200 + 8*n, n=0...127; RW)
uint32_t ebbrt::IxgbeDriver::ReadRal(uint32_t n) {
  auto reg = bar0_.Read32(0x0A200 + 8 * n);
  return reg;
}
void ebbrt::IxgbeDriver::WriteRal(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0A200 + (8 * n), m);
}

// 8.2.3.7.9 Receive Address High — RAH[n] (0x0A204 + 8*n, n=0...127; RW)
uint16_t ebbrt::IxgbeDriver::ReadRah(uint32_t n) {
  auto reg = bar0_.Read32(0x0A204 + 8 * n);
  return (reg)&0xFFFF;
}
uint8_t ebbrt::IxgbeDriver::ReadRahAv(uint32_t n) {
  return (bar0_.Read32(0x0A204 + 8 * n) >> 31) & 0xFF;
}
void ebbrt::IxgbeDriver::WriteRah(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0A204 + (8 * n), m);
}

// 8.2.3.7.10 MAC Pool Select Array — MPSAR[n] (0x0A600 + 4*n, n=0...255; RW)
void ebbrt::IxgbeDriver::WriteMpsar(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0A600 + 4 * n, m);
}

// 8.2.3.7.19 Five tuple Queue Filter — FTQF[n] (0x0E600 + 4*n,n=0...127; RW)
void ebbrt::IxgbeDriver::WriteFtqf(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0E600 + 4 * n, m);
}

// 8.2.3.7.16 Source Address Queue Filter — SAQF[n] (0x0E000 + 4*n, n=0...127;
// RW)
void ebbrt::IxgbeDriver::WriteSaqf(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0E000 + 4 * n, m);
}

// 8.2.3.7.17 Destination Address Queue Filter — DAQF[n] (0x0E200 + 4*n,
// n=0...127; RW)
void ebbrt::IxgbeDriver::WriteDaqf(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0E200 + 4 * n, m);
}

// 8.2.3.7.18 Source Destination Port Queue Filter — SDPQF[n] (0x0E400 + 4*n,
// n=0...127; RW)
void ebbrt::IxgbeDriver::WriteSdpqf(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0E400 + 4 * n, m);
}

// 8.2.3.27.17 PF Unicast Table Array — PFUTA[n] (0x0F400 + 4*n, n=0...127; RW)
void ebbrt::IxgbeDriver::WritePfuta(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0F400 + 4 * n, m);
}

// 8.2.3.7.3 Multicast Control Register — MCSTCTRL (0x05090; RW)
void ebbrt::IxgbeDriver::WriteMcstctrl(uint32_t m) {
  auto reg = bar0_.Read32(0x05090);
  bar0_.Write32(0x05090, reg & m);
}

// 8.2.3.10.13 DCB Transmit Descriptor Plane Queue Select — RTTDQSEL (0x04904;
// RW)
void ebbrt::IxgbeDriver::WriteRttdqsel(uint32_t m) {
  auto reg = bar0_.Read32(0x04904);
  bar0_.Write32(0x04904, reg | m);
}

// 8.2.3.10.14 DCB Transmit Descriptor Plane T1 Config — RTTDT1C (0x04908; RW)
void ebbrt::IxgbeDriver::WriteRttdt1c(uint32_t m) { bar0_.Write32(0x04908, m); }

// 8.2.3.10.16 DCB Transmit Rate-Scheduler Config — RTTBCNRC (0x04984; RW)
void ebbrt::IxgbeDriver::WriteRttbcnrc(uint32_t m) {
  bar0_.Write32(0x04984, m);
}

// 8.2.3.10.9 DCB Transmit Descriptor Plane T2 Config - RTTDT2C[n] (0x04910 +
// 4*n, n=0...7; RW) DMA-Tx
void ebbrt::IxgbeDriver::WriteRttdt2c(uint32_t n, uint32_t m) {
  bar0_.Write32(0x04910 + 4 * n, m);
}

// 8.2.3.10.10 DCB Transmit Packet Plane T2 Config — RTTPT2C[n] (0x0CD20 + 4*n,
// n=0...7; RW)
void ebbrt::IxgbeDriver::WriteRttpt2c(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0CD20 + 4 * n, m);
}

// 8.2.3.10.6 DCB Receive Packet Plane T4 Config — RTRPT4C[n] (0x02140 + 4*n,
// n=0...7; RW)
void ebbrt::IxgbeDriver::WriteRtrpt4c(uint32_t n, uint32_t m) {
  bar0_.Write32(0x02140 + 4 * n, m);
}

// 8.2.3.10.1 DCB Receive Packet Plane Control and Status — RTRPCS (0x02430; RW)
void ebbrt::IxgbeDriver::WriteRtrpcs(uint32_t m) { bar0_.Write32(0x02430, m); }

// 8.2.3.11.2 Tx DCA Control Registers — DCA_TXCTRL[n] (0x0600C + 0x40*n,
// n=0...127; RW)
void ebbrt::IxgbeDriver::WriteDcaTxctrlTxdescWbro(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x0600C + 0x40 * n);
  bar0_.Write32(0x0600C + 0x40 * n, reg & m);
}
void ebbrt::IxgbeDriver::ReadDcaTxctrl(uint32_t n) {
  auto reg = bar0_.Read32(0x0600C + 0x40 * n);
  ebbrt::kprintf("DCA_TXCTRL=0x%X\n", reg);
}

// 8.2.3.11.1 Rx DCA Control Register — DCA_RXCTRL[n] (0x0100C + 0x40*n,
// n=0...63 and 0x0D00C + 0x40*(n-64),
// n=64...127 / 0x02200 + 4*n, [n=0...15]; RW)
void ebbrt::IxgbeDriver::WriteDcaRxctrl_1(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x0100C + 0x40 * n);
  bar0_.Write32(0x0100C + 0x40 * n, reg & m);
}
void ebbrt::IxgbeDriver::ReadDcaRxctrl(uint32_t n) {
  auto reg = bar0_.Read32(0x0100C + 0x40 * n);
  ebbrt::kprintf("DCA_RXCTRL=0x%X\n", reg);
}

void ebbrt::IxgbeDriver::WriteDcaRxctrl_2(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x0D00C + 0x40 * n);
  bar0_.Write32(0x0D00C + 0x40 * n, reg & m);
}

// 8.2.3.7.5 Receive Checksum Control — RXCSUM (0x05000; RW)
void ebbrt::IxgbeDriver::WriteRxcsum(uint32_t m) {
  //auto reg = bar0_.Read32(0x05000);
  //bar0_.Write32(0x05000, reg | m);
  bar0_.Write32(0x05000, m);
}

// 8.2.3.8.13 RSC Control — RSCCTL[n] (0x0102C + 0x40*n, n=0...63
// and 0x0D02C + 0x40*(n-64), n=64...127; RW)
void ebbrt::IxgbeDriver::WriteRscctl(uint32_t n, uint32_t m) {
  //auto reg = bar0_.Read32(0x0102C + 0x40 * n);
  //bar0_.Write32(0x0102C + 0x40 * n, reg | m);
  bar0_.Write32(0x0102C + 0x40 * n, m);
}

// 8.2.3.7.4 Packet Split Receive Type Register — PSRTYPE[n]
// (0x0EA00 + 4*n, n=0...63 / 0x05480 + 4*n, n=0...15; RW)
void ebbrt::IxgbeDriver::WritePsrtype(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0EA00 + 4 * n, m);
}

void ebbrt::IxgbeDriver::WritePsrtypeZero(uint32_t n) {
  //bar0_.Write32(0x0EA00 + 0x40 * n, 0x0);
  bar0_.Write32(0x05480, n);
}

// 8.2.3.7.15 Redirection Table — RETA[n] (0x0EB00 + 4*n, n=0...31/ 0x05C00 +
// 4*n, n=0...31; RW)
void ebbrt::IxgbeDriver::WriteReta(uint32_t n, uint32_t m) {
  //bar0_.Write32(0x0EB00 + 4 * n, m);
  bar0_.Write32(0x05C00 + 4 * n, m);
  ebbrt::kprintf("WriteReta(n=%d) %X = 0x%08X\n", n, 0x05C00 + 4 * n, m);
}

// 8.2.3.7.6 Receive Filter Control Register — RFCTL (0x05008; RW)
void ebbrt::IxgbeDriver::WriteRfctl(uint32_t m) { bar0_.Write32(0x05008, m); }

// 8.2.3.9.16 Tx Packet Buffer Threshold —
// TXPBTHRESH (0x04950 +0x4*n, n=0...7; RW)
void ebbrt::IxgbeDriver::WriteTxpbthresh(uint32_t n, uint32_t m) {
  bar0_.Write32(0x04950 + 0x4 * n, m);
}

// 8.2.3.7.12 Multiple Receive Queues Command Register- MRQC (0x0EC80 / 0x05818;
// RW)
void ebbrt::IxgbeDriver::WriteMrqc(uint32_t m) {
  //auto reg = bar0_.Read32(0x0EC80);
  //bar0_.Write32(0x0EC80, reg | m);
  bar0_.Write32(0x05818, m);
}

// 8.2.3.9.15 Multiple Transmit Queues Command Register — MTQC (0x08120; RW)
void ebbrt::IxgbeDriver::WriteMtqc(uint32_t m) { bar0_.Write32(0x08120, m); }

// 8.2.3.27.1 VT Control Register — PFVTCTL (0x051B0; RW)
void ebbrt::IxgbeDriver::WritePfvtctl(uint32_t m) { bar0_.Write32(0x051B0, m); }

// 8.2.3.10.4 DCB Receive User Priority to Traffic Class — RTRUP2TC (0x03020;
// RW)
void ebbrt::IxgbeDriver::WriteRtrup2tc(uint32_t m) {
  bar0_.Write32(0x03020, m);
}

// 8.2.3.10.5 DCB Transmit User Priority to Traffic Class — RTTUP2TC (0x0C800;
// RW)
void ebbrt::IxgbeDriver::WriteRttup2tc(uint32_t m) {
  bar0_.Write32(0x0C800, m);
}

// 8.2.3.9.1 DMA Tx TCP Max Allow Size Requests — DTXMXSZRQ (0x08100; RW)
void ebbrt::IxgbeDriver::WriteDtxmxszrq(uint32_t m) {
  auto reg = bar0_.Read32(0x08100);
  bar0_.Write32(0x08100, reg | m);
}

// 8.2.3.27.9 PF PF Queue Drop Enable Register — PFQDE (0x02F04; RW)
void ebbrt::IxgbeDriver::WritePfqde(uint32_t m) { bar0_.Write32(0x02F04, m); }

// 8.2.3.22.34 MAC Flow Control Register — MFLCN (0x04294; RW)
void ebbrt::IxgbeDriver::WriteMflcn(uint32_t m) {
  auto reg = bar0_.Read32(0x04294);
  bar0_.Write32(0x04294, reg | m);
}

// 8.2.3.3.7 Flow Control Configuration — FCCFG (0x03D00; RW)
/*void ebbrt::IxgbeDriver::WriteFccfg(uint32_t m) {
  auto reg = bar0_.Read32(0x03D00);
  bar0_.Write32(0x03D00, reg | m);
  }*/

// void ebbrt::IxgbeDriver::WriteDcaRxctrl_2_RxdataWrro(uint32_t n, uint32_t m);

// 8.2.3.4.9 - Software Semaphore Register — SWSM (0x10140; RW)
bool ebbrt::IxgbeDriver::SwsmSmbiRead() {
  return !!(bar0_.Read32(0x10140) & 0x1);
}
bool ebbrt::IxgbeDriver::SwsmSwesmbiRead() {
  return !(bar0_.Read32(0x10140) & 0x2);
}
void ebbrt::IxgbeDriver::SwsmSwesmbiSet() {
  auto reg = bar0_.Read32(0x10140);
  ebbrt::kprintf("%s: reg before: 0x%08X, reg after: 0x%08X\n", __FUNCTION__,
                 reg, reg | 0x2);
  bar0_.Write32(0x10140, reg | 0x2);
}
void ebbrt::IxgbeDriver::SwsmSmbiClear() {
  auto reg = bar0_.Read32(0x10140);
  ebbrt::kprintf("%s: reg before: 0x%08X, reg after: 0x%08X\n", __FUNCTION__,
                 reg, reg & 0xFFFFFFFE);
  bar0_.Write32(0x10140, reg & 0xFFFFFFFE);
}
void ebbrt::IxgbeDriver::SwsmSwesmbiClear() {
  auto reg = bar0_.Read32(0x10140);
  ebbrt::kprintf("%s: reg before: 0x%08X, reg after: 0x%08X\n", __FUNCTION__,
                 reg, reg & 0xFFFFFFFD);
  bar0_.Write32(0x10140, reg & 0xFFFFFFFD);
}

// 8.2.3.22.20 Link Status Register — LINKS (0x042A4; RO)
bool ebbrt::IxgbeDriver::ReadLinksLinkUp() {
  auto reg = bar0_.Read32(0x042A4);
  return ((reg >> 30) & 0x1) == 1;
}

// 8.2.3.4.11 Software-Firmware Synchronization - SW_FW_SYNC (0x10160; RW)
uint32_t ebbrt::IxgbeDriver::ReadSwfwSyncSmBits(uint32_t m) {
  auto reg = bar0_.Read32(0x10160);
  return (reg & m) & 0x3FF;  // masking bits 9:0
}
void ebbrt::IxgbeDriver::WriteSwfwSyncSmBits(uint32_t m) {
  auto reg = bar0_.Read32(0x10160);
  bar0_.Write32(0x10160, reg | m);
}
void ebbrt::IxgbeDriver::WriteSwfwSyncSmBits2(uint32_t m) {
  auto reg = bar0_.Read32(0x10160);
  bar0_.Write32(0x10160, reg & m);
}

// 8.2.3.11.1 Rx DCA Control Register — DCA_RXCTRL[n] (0x0100C + 0x40*n,
// n=0...63 and 0x0D00C + 0x40*(n-64), // n=0...63 and 0x0D00C + 0x40*(n-64),
// n=64...127 / 0x02200 + 4*n, [n=0...15]; RW) // n=64...127 / 0x02200 + 4*n,
// [n=0...15]; RW)
void ebbrt::IxgbeDriver::WriteDcaRxctrl(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x0100C + 0x40 * n);
  bar0_.Write32(0x0100C + 0x40 * n, reg | m);
}
void ebbrt::IxgbeDriver::WriteDcaRxctrlClear(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x0100C + 0x40 * n);
  bar0_.Write32(0x0100C + 0x40 * n, reg & m);
}

// 8.2.3.11.4 DCA Control Register — DCA_CTRL (0x11074; RW)
void ebbrt::IxgbeDriver::WriteDcaCtrl(uint32_t m) {
  auto reg = bar0_.Read32(0x11074);
  bar0_.Write32(0x11074, reg | m);
}

// 8.2.3.11.2 Tx DCA Control Registers — DCA_TXCTRL[n] (0x0600C + 0x40*n,
// n=0...127; RW)
void ebbrt::IxgbeDriver::WriteDcaTxctrl(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x0600C + 0x40 * n);
  bar0_.Write32(0x0600C + 0x40 * n, reg | m);
}

// 8.2.3.8.1 Receive Descriptor Base Address Low — RDBAL[n] (0x01000 + 0x40*n,
// n=0...63 and 0x0D000 + 0x40*(n-64), n=64...127; RW)
void ebbrt::IxgbeDriver::WriteRdbal_1(uint32_t n, uint32_t m) {
  bar0_.Write32(0x01000 + 0x40 * n, m);
}
void ebbrt::IxgbeDriver::WriteRdbal_2(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0D000 + 0x40 * n, m);
}

// 8.2.3.8.2 Receive Descriptor Base Address High — RDBAH[n] (0x01004 + 0x40*n,
// n=0...63 and 0x0D004 + 0x40*(n-64), n=64...127; RW)
void ebbrt::IxgbeDriver::WriteRdbah_1(uint32_t n, uint32_t m) {
  bar0_.Write32(0x01004 + 0x40 * n, m);
}
void ebbrt::IxgbeDriver::WriteRdbah_2(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0D004 + 0x40 * n, m);
}

// 8.2.3.9.5 Transmit Descriptor Base Address Low — TDBAL[n] (0x06000+0x40*n,
// n=0...127; RW)
void ebbrt::IxgbeDriver::WriteTdbal(uint32_t n, uint32_t m) {
  bar0_.Write32(0x06000 + 0x40 * n, m);
}

// 8.2.3.9.6 Transmit Descriptor Base Address High — TDBAH[n] (0x06004+0x40*n,
// n=0...127; RW)
void ebbrt::IxgbeDriver::WriteTdbah(uint32_t n, uint32_t m) {
  bar0_.Write32(0x06004 + 0x40 * n, m);
}

// 8.2.3.9.7 Transmit Descriptor Length — TDLEN[n] (0x06008+0x40*n, n=0...127;
// RW)
void ebbrt::IxgbeDriver::WriteTdlen(uint32_t n, uint32_t m) {
  bar0_.Write32(0x06008 + 0x40 * n, m);
}

// 8.2.3.9.8 Transmit Descriptor Head — TDH[n] (0x06010+0x40*n, n=0...127; RO)
void ebbrt::IxgbeDriver::WriteTdh(uint32_t n, uint32_t m) {
  bar0_.Write32(0x06010 + 0x40 * n, m);
}
uint16_t ebbrt::IxgbeDriver::ReadTdh(uint32_t n) {
  auto reg = bar0_.Read32(0x06010 + 0x40 * n);
  return reg & 0xFFFF;
}
uint32_t ebbrt::IxgbeDriver::ReadTdt(uint32_t n) {
  return bar0_.Read32(0x06018 + 0x40 * n) & 0xFFFF;
}

// 8.2.3.9.11 Tx Descriptor Completion Write Back Address Low —
// TDWBAL[n] (0x06038+0x40*n, n=0...127; RW)
void ebbrt::IxgbeDriver::WriteTdwbal(uint32_t n, uint32_t m) {
  bar0_.Write32(0x06038 + 0x40 * n, m);
}
// 8.2.3.9.12 Tx Descriptor Completion Write Back Address High —
// TDWBAH[n] (0x0603C+0x40*n, n=0...127; RW)
void ebbrt::IxgbeDriver::WriteTdwbah(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0603C + 0x40 * n, m);
}

// 8.2.3.9.9 Transmit Descriptor Tail — TDT[n] (0x06018+0x40*n, n=0...127; RW)
void ebbrt::IxgbeDriver::WriteTdt(uint32_t n, uint32_t m) {
  bar0_.Write32(0x06018 + 0x40 * n, m);
}

// 8.2.3.8.3 Receive Descriptor Length — RDLEN[n] (0x01008 + 0x40*n, n=0...63
// and 0x0D008 + 0x40*(n-64), n=64...127; RW)
void ebbrt::IxgbeDriver::WriteRdlen_1(uint32_t n, uint32_t m) {
  bar0_.Write32(0x01008 + 0x40 * n, m);
}
void ebbrt::IxgbeDriver::WriteRdlen_2(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0D008 + 0x40 * n, m);
}

// 8.2.3.8.7 Split Receive Control Registers — SRRCTL[n] (0x01014 + 0x40*n,
// n=0...63 and 0x0D014 + 0x40*(n-64), n=64...127 / 0x02100 + 4*n, [n=0...15];
// RW)
void ebbrt::IxgbeDriver::WriteSrrctl_1(uint32_t n, uint32_t m) {
  //auto reg = bar0_.Read32(0x01014 + 0x40 * n);
  bar0_.Write32(0x01014 + 0x40 * n, m);
}
void ebbrt::IxgbeDriver::WriteSrrctlZero(uint32_t n) {
  bar0_.Write32(0x01014 + 0x40 * n, 0x0);
}

// 8.2.3.8.12 RSC Data Buffer Control Register — RSCDBU (0x03028; RW)
void ebbrt::IxgbeDriver::WriteRscdbu(uint32_t m) {
  //auto reg = bar0_.Read32(0x03028);
  bar0_.Write32(0x03028, m);
}

void ebbrt::IxgbeDriver::WriteSrrctl_1_desctype(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x01014 + 0x40 * n);
  bar0_.Write32(0x01014 + 0x40 * n, reg & m);
}

// 8.2.3.8.8 Receive DMA Control Register — RDRXCTL (0x02F00; RW)
void ebbrt::IxgbeDriver::WriteRdrxctl(uint32_t m) {
  auto reg = bar0_.Read32(0x02F00);
  bar0_.Write32(0x02F00, reg | m);
}

void ebbrt::IxgbeDriver::WriteRdrxctlRSCFRSTSIZE(uint32_t m) {
  auto reg = bar0_.Read32(0x02F00);
  bar0_.Write32(0x02F00, reg & m);
}

uint8_t ebbrt::IxgbeDriver::ReadRdrxctlDmaidone() {
  auto reg = bar0_.Read32(0x02F00);
  return (reg >> 3) & 0x1;
}

// 8.2.3.8.9 Receive Packet Buffer Size — RXPBSIZE[n] (0x03C00 + 4*n, n=0...7;
// RW)
void ebbrt::IxgbeDriver::WriteRxpbsize(uint32_t n, uint32_t m) {
  bar0_.Write32(0x03C00 + 4 * n, m);
}

// 8.2.3.9.13 Transmit Packet Buffer Size — TXPBSIZE[n] (0x0CC00 + 0x4*n,
// n=0...7; RW)
void ebbrt::IxgbeDriver::WriteTxpbsize(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0CC00 + 0x4 * n, m);
}

// 8.2.3.9.16 Tx Packet Buffer Threshold — TXPBTHRESH (0x04950+0x4*n, n=0...7;
// RW)
void ebbrt::IxgbeDriver::WriteTxpbThresh(uint32_t n, uint32_t m) {
  bar0_.Write32(0x04950 + 0x4 * n, m);
}

// 8.2.3.22.8 MAC Core Control 0 Register — HLREG0 (0x04240; RW)
void ebbrt::IxgbeDriver::WriteHlreg0(uint32_t m) {
  //auto reg = bar0_.Read32(0x04240);
  bar0_.Write32(0x04240, m);
}

// 8.2.3.22.13 Max Frame Size — MAXFRS (0x04268; RW)
void ebbrt::IxgbeDriver::WriteMaxfrs(uint32_t m) {
  bar0_.Write32(0x04268, m);
}

// 8.2.3.8.5 Receive Descriptor Tail — RDT[n] (0x01018 + 0x40*n, n=0...63 and
// 0x0D018 + 0x40*(n-64), n=64...127; RW)
void ebbrt::IxgbeDriver::WriteRdt_1(uint32_t n, uint32_t m) {
  bar0_.Write32(0x01018 + 0x40 * n, m);
}
void ebbrt::IxgbeDriver::WriteRdt_2(uint32_t n, uint32_t m) {
  bar0_.Write32(0x0D018 + 0x40 * n, m);
}

// 8.2.3.8.4 Receive Descriptor Head — RDH[n] (0x01010 + 0x40*n, n=0...63 and
// 0x0D010 + 0x40*(n-64), n=64...127; RO)
void ebbrt::IxgbeDriver::WriteRdh_1(uint32_t n, uint32_t m) {
  bar0_.Write32(0x01010 + 0x40 * n, m);
}
void ebbrt::IxgbeDriverRep::WriteRdh_1(uint32_t n, uint32_t m) {
  root_.bar0_.Write32(0x01010 + 0x40 * n, m);
}

uint16_t ebbrt::IxgbeDriver::ReadRdh_1(uint32_t n) {
  auto reg = bar0_.Read32(0x01010 + 0x40 * n);
  return reg & 0xFFFF;
}

uint16_t ebbrt::IxgbeDriver::ReadRdt_1(uint32_t n) {
  auto reg = bar0_.Read32(0x01018 + 0x40 * n);
  return reg & 0xFFFF;
}

void ebbrt::IxgbeDriver::SwfwSemRelease() {
  SwsmSwesmbiClear();
  SwsmSmbiClear();
  ebbrt::kprintf("%s\n", __FUNCTION__);
}

// 8.2.3.5.16 Interrupt Vector Allocation Registers — IVAR[n] (0x00900 + 4*n,
// n=0...63; RW)
void ebbrt::IxgbeDriver::WriteIvarAlloc0(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x00900 + 4 * n);
  bar0_.Write32(0x00900 + 4 * n, reg | m);
  ebbrt::kprintf("IVAR: 0x%X 0x%X\n", 0x00900 + 4 * n, reg | m);
}
void ebbrt::IxgbeDriver::WriteIvarAllocval0(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x00900 + 4 * n);
  bar0_.Write32(0x00900 + 4 * n, reg | m);
  ebbrt::kprintf("IVAR: 0x%X 0x%X\n", 0x00900 + 4 * n, reg | m);
}

void ebbrt::IxgbeDriver::WriteIvarAlloc1(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x00900 + 4 * n);
  bar0_.Write32(0x00900 + 4 * n, reg | m);
}
void ebbrt::IxgbeDriver::WriteIvarAllocval1(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x00900 + 4 * n);
  bar0_.Write32(0x00900 + 4 * n, reg | m);
}

void ebbrt::IxgbeDriver::WriteIvarAlloc2(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x00900 + 4 * n);
  bar0_.Write32(0x00900 + 4 * n, reg | m);
  ebbrt::kprintf("IVAR: 0x%X 0x%X\n", 0x00900 + 4 * n, reg | m);
}
void ebbrt::IxgbeDriver::WriteIvarAllocval2(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x00900 + 4 * n);
  bar0_.Write32(0x00900 + 4 * n, reg | m);
  ebbrt::kprintf("IVAR: 0x%X 0x%X\n", 0x00900 + 4 * n, reg | m);
}

void ebbrt::IxgbeDriver::WriteIvarAlloc3(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x00900 + 4 * n);
  bar0_.Write32(0x00900 + 4 * n, reg | m);
}
void ebbrt::IxgbeDriver::WriteIvarAllocval3(uint32_t n, uint32_t m) {
  auto reg = bar0_.Read32(0x00900 + 4 * n);
  bar0_.Write32(0x00900 + 4 * n, reg | m);
}

// 8.2.3.10.2 DCB Transmit Descriptor Plane Control and Status — RTTDCS
// (0x04900; RW) DMA-Tx
void ebbrt::IxgbeDriver::WriteRttdcs(uint32_t m) {
  //auto reg = bar0_.Read32(0x04900);
  bar0_.Write32(0x04900, m);
}
void ebbrt::IxgbeDriver::WriteRttdcsArbdisEn(uint32_t m) {
  auto reg = bar0_.Read32(0x04900);
  bar0_.Write32(0x04900, reg & m);
}

// 8.2.3.10.3 DCB Transmit Packet Plane Control and Status- RTTPCS (0x0CD00; RW)
void ebbrt::IxgbeDriver::WriteRttpcs(uint32_t m) { bar0_.Write32(0x0CD00, m); }

// 8.2.3.12.5 Security Rx Control — SECRXCTRL (0x08D00; RW)
void ebbrt::IxgbeDriver::WriteSecrxctrl_Rx_Dis(uint32_t m) {
  bar0_.Write32(0x08D00, m);
  /*auto reg = bar0_.Read32(0x08D00);
  if (m) {
    bar0_.Write32(0x08D00, reg | m);
  } else {
    bar0_.Write32(0x08D00, reg & ~(0x1 << 1));
    }*/
}

// 8.2.3.12.6 Security Rx Status — SECRXSTAT (0x08D04; RO)
uint8_t ebbrt::IxgbeDriver::ReadSecrxstat_Sr_Rdy() {
  auto reg = bar0_.Read32(0x08D04);
  return reg & 0x1;
}

// 8.2.3.23.59 Total Packets Received — TPR (0x040D0; RC)
uint32_t ebbrt::IxgbeDriver::ReadTpr() {
  auto reg = bar0_.Read32(0x040D0);
  ebbrt::kprintf("%s %d\n", __FUNCTION__, reg);
  return reg;
}

// 8.2.3.23.26 Good Packets Received Count — GPRC (0x04074; RO)
uint32_t ebbrt::IxgbeDriver::ReadGprc() {
  auto reg = bar0_.Read32(0x04074);
  ebbrt::kprintf("%s %d\n", __FUNCTION__, reg);
  return reg;
}

bool ebbrt::IxgbeDriver::SwfwSemAcquire() {
  // polls SWSM.SMBI until 0b is read or timeout
  // TODO: timeout after 10 ms
  while (SwsmSmbiRead())
    ;

  // writes 1b to SWSM.SWESMBI bit
  SwsmSwesmbiSet();

  // polls SWSM.SWESMBI bit until read as 1b
  // TODO: timeout of 3 secs
  while (SwsmSwesmbiRead())
    ;

  return true;
}

// 10.5.4 Software and Firmware Synchronization
bool ebbrt::IxgbeDriver::SwfwLockPhy() {
  bool good = false;

again:
  if (!SwfwSemAcquire()) {
    ebbrt::kabort("SwfwSemAcquire failed\n");
  } else {
    ebbrt::kprintf("SWSM Sem acquired\n");
  }

  if ((ReadStatusLanId() == 0) && (ReadSwfwSyncSmBits(0x2) == 0)  // SW_PHY_SM0
      && (ReadSwfwSyncSmBits(0x40) == 0))  // FW_PHY_SM0
  {
    WriteSwfwSyncSmBits(0x2);  // SW_PHY_SM0
    ebbrt::kprintf("SW_PHY_SMO written\n");
    good = true;
  } else if ((ReadSwfwSyncSmBits(0x4) == 0)  // SW_PHY_SM1
             && (ReadSwfwSyncSmBits(0x80) == 0))  // FW_PHY_SM1
  {
    WriteSwfwSyncSmBits(0x4);  // SW_PHY_SM1
    ebbrt::kprintf("SW_PHY_SM1 written\n");
    good = true;
  }

  SwfwSemRelease();

  if (!good) {
    ebbrt::kprintf("%s: failed, trying again\n", __FUNCTION__);
    ebbrt::clock::SleepMilli(20);
    goto again;
  }

  return true;
}
void ebbrt::IxgbeDriver::SwfwUnlockPhy() {
  if (!SwfwSemAcquire()) {
    ebbrt::kabort("SwfwSemAcquire failed\n");
  } else {
    ebbrt::kprintf("SWSM Sem acquired\n");
  }

  if (ReadStatusLanId() == 0) {
    WriteSwfwSyncSmBits2(~0x2);  // SW_PHY_SM0
  } else {
    WriteSwfwSyncSmBits2(~0x4);  // SW_PHY_SM1
  }

  SwfwSemRelease();

  ebbrt::clock::SleepMilli(10);
}

void ebbrt::IxgbeDriver::StopDevice() {
  ebbrt::kprintf("%s ", __PRETTY_FUNCTION__);

  // disable rx
  WriteRxctrl(0x0);

  // disable tx
  WriteDmatxctl(0xFFFFFFFE);

  // disable interrupts
  //WriteEimc(0x7FFFFFFF);
  WriteEimc(0xFFFFFFFF);
  ReadEicr();

  // disable each rx and tx queue
  for (auto i = 0; i < 128; i++) {
    // Bit 26, transmit software flush
    WriteTxdctl(i, 0x04000000);

    if (i < 64) {
      WriteRxdctl_1(i, 0x0);
    } else {
      WriteRxdctl_2(i - 64, 0x0);
    }
  }

  // from arrakis
  ebbrt::clock::SleepMilli(2);

  // Master disable procedure
  WriteCtrl(0x4);  // PCIe Master Disable
  while (ReadStatusPcieMes() != 1)
    ;
  ebbrt::kprintf("Ixgbe 82599 stop done\n");
}

void ebbrt::IxgbeDriver::GlobalReset() {
  ebbrt::kprintf("%s ", __PRETTY_FUNCTION__);

  WriteCtrl(0x8);  // Link Reset
  WriteCtrl(0x4000000);  // Device Reset

  // Note: To ensure that a global device reset has fully completed and that the
  // 82599 responds to  subsequent accesses, programmers must wait
  // before  approximately 1 ms after setting attempting to check
  // if the bit has cleared or to access (read or write) any other device
  // register.
  ebbrt::clock::SleepMilli(2);
  ReadCtrl();
}

/**
 *  ixgbe_init_hw_generic - Generic hardware initialization
 *  @hw: pointer to hardware structure
 *
 *  Initialize the hardware by resetting the hardware, filling the bus info
 *  structure and media type, clears all on chip counters, initializes receive
 *  address registers, multicast table, VLAN filter table, calls routine to set
 *  up link and flow control settings, and leaves transmit and receive units
 *  disabled and uninitialized
 **/
void ebbrt::IxgbeDriver::Init() {
  uint64_t d_mac;
  uint32_t ncore = static_cast<uint32_t>(Cpu::Count());

  //ebbrt::kprintf("%s ncore=%u", __PRETTY_FUNCTION__, ncore);
  bar0_.Map();  // allocate virtual memory
  ebbrt::clock::SleepMilli(200);
  ebbrt::kprintf("Sleep 200 ms\n");

  StopDevice();
  GlobalReset();
  ebbrt::clock::SleepMilli(50);
  GlobalReset();
  ebbrt::clock::SleepMilli(250);

  // disable interrupts
  //WriteEimc(0x7FFFFFFF);
  WriteEimc(0xFFFFFFFF);
  ReadEicr();

  // Let firmware know we have taken over
  //WriteCtrlExt(0x1 << 28);  // DRV_LOAD
  WriteCtrlExt(0x10010000);  // DRV_LOAD and NS_DIS

  //If legacy descriptors are used, this bit should be set to 1b.
  // No snoop disable from FreeBSD ??
//#ifndef RSC_EN
//  WriteCtrlExt(0x1 << 16);  // NS_DIS
//#endif
  
  // Initialize flow-control registers
  for (auto i = 0; i < 8; i++) {
    if (i < 4) {
      WriteFcttv(i, 0x0);
    }
    WriteFcrtl(i, 0x0);
    WriteFcrth(i, 0x0);
  }

  WriteFcrtv(0x0);
  WriteFccfg(0x0);

  // Initialize Phy
  PhyInit();

  // Wait for EEPROM auto read
  while (ReadEecAutoRd() == 0) {
  };  // TODO: Timeout
  ebbrt::kprintf("EEPROM auto read done\n");

  ebbrt::clock::SleepMilli(200);
  d_mac = ReadRal(0) | ((uint64_t)ReadRah(0) << 32);
  // ebbrt::kprintf("mac %p valid = %x\n", d_mac, ReadRahAv(0));
  for (auto i = 0; i < 6; i++) {
    mac_addr_[i] = (d_mac >> (i * 8)) & 0xFF;
  }
  ebbrt::kprintf(
      "Mac Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
      static_cast<uint8_t>(mac_addr_[0]), static_cast<uint8_t>(mac_addr_[1]),
      static_cast<uint8_t>(mac_addr_[2]), static_cast<uint8_t>(mac_addr_[3]),
      static_cast<uint8_t>(mac_addr_[4]), static_cast<uint8_t>(mac_addr_[5]));

  // Wait for DMA initialization
  while (ReadRdrxctlDmaidone() == 0) {
  };  // TODO: Timeout

  // Wait for link to come up
  while (!ReadLinksLinkUp()) {
  };  // TODO: timeout
  ebbrt::kprintf("Link is up\n");
  ebbrt::clock::SleepMilli(50);

  // clears on read
  WriteEicr(0xFFFFFFFF);

  /*
   * use EIAM to auto-mask when MSI-X interrupt is asserted
   * this saves a register write for every interrupt
   */
  //WriteEiam(0, 0xFFFFFFFF);
  //WriteEiam(1, 0xFFFFFFFF);
  
  /* setup msix */
  // switch to msix mode
  WriteGpie(0x1 << 4 | 0x1 << 5 | 0x1 << 31 | 0x1 << 30);  // Multiple_MSIX  
  //WriteGpie(0x1 << 5);  // OCD
  //WriteGpie(0x1 << 31);  // PBA_support
  // Enable auto masking of interrupt
  //WriteGpie(0x1 << 30);  // EIAME
  
  // TODO: Set up management interrupt handler

  //WriteGpie(0xC0000036);
  //uint32_t gpie = 0xC0000036 | (0x7 << 11);
  //uint32_t gpie = 0xC0000036 | (IxgbeDriver::RSC_DELAY << 11);
  //WriteGpie(gpie);
  //ebbrt::kprintf_force("RSC enabled, RSC_DELAY = %d us\n", (((gpie >> 11) & 0x7)+1)*4);
  
#ifdef RSC_EN
  // TODO: RSC delay value, just a guess at (1 + 1) * 4us = 8 us
  // Rexcommended value based on 7.3.2.1.1
  WriteGpie(0x1 << 4 | 0x1 << 5 | 0x1 << 31 | 0x1 << 30 | IxgbeDriver::RSC_DELAY << 11);
  ebbrt::kprintf_force("RSC enabled, RSC_DELAY = %d\n", (IxgbeDriver::RSC_DELAY + 1) * 4);
#endif

  /* FreeBSD:
   * ixgbe_common.c - s32 ixgbe_init_rx_addrs_generic(struct ixgbe_hw *hw)
   * Places the MAC address in receive address register 0 and clears the rest
   *  of the receive address registers. Clears the multicast table. Assumes
   *  the receiver is in reset when the routine is called.
   */
  // Initialize RX filters

  /* Zero out the other receive addresses. */
  for (auto i = 1; i < 128; i++) {
    WriteRal(i, 0x0);
    WriteRah(i, 0x0);
  }

  // clear mta
  for (auto i = 0; i < 128; i++) {
    WriteMta(i, 0x0);
  }

  // No init uta tables?

  // set vlan filter table
  for (auto i = 0; i < 128; i++) {
    WriteVfta(i, 0x0);
  }

  for (auto i = 0; i < 64; i++) {
    // WritePfvlvf(i, 0x1 << 31);  // VI_En bit 31
    WritePfvlvf(i, 0x0);
    WritePfvlvfb(i, 0x0);
    // WritePsrtypeZero(0x0);
  }

  // PF Unicast Table Array
  for (auto i = 0; i < 128; i++) {
    WritePfuta(i, 0x0);
  }

  // not sure why initing these tables?
  for (auto i = 0; i < 128; i++) {
    WriteFhft_1(i, 0x0);
    if (i < 64) {
      WriteFhft_2(i, 0x0);
    }
  }

  // enable ECC Reporting TODO - causes interrupts to be broken??
  // WriteRxfeccerr0(0x1 << 9);

  /**** Initialize RX filters ****/
  // FreeBSD if_ix.c - ixgbe_initialize_receive_units - Enable broadcast accept
  WriteFctrl(0x1 << 10);  // Set BAM = 1

  // TODO VLNCTRL
  WriteMcstctrl(0x0);

//#ifndef RSC_EN
  //WriteRxcsum(0x1 << 12);  // IP payload checksum enable
  //WriteRxcsum(0x3 << 12);  // IP payload checksum enable
  WriteRxcsum(0x3000);
//#else
  // note: PCSD: The Fragment Checksum and IP Identification fields are mutually exclusive with
  // the RSS hash. Only one of the two options is reported in the Rx descriptor.
//  WriteRxcsum(0x2000);
//#endif
// TODO RQTC

#ifdef RSC_EN
  WriteRfctl(0xC0);
#else
  WriteRfctl(0xE0);
#endif

  for (auto i = 0; i < 256; i++) {
    WriteMpsar(i, 0x0);
  }

  // !! Support for RSS is not provided when legacy receive descriptor format is used.
  
  // RSSRK - random seeds taken from Linux
  WriteRssrk(0, 0xA38DD80F);
  WriteRssrk(1, 0xD107C3DC);
  WriteRssrk(2, 0x8CEB19C4);
  WriteRssrk(3, 0xA41E1B6B);
  WriteRssrk(4, 0xB7218638);
  WriteRssrk(5, 0x6B8B6155);
  WriteRssrk(6, 0xDC8D08B5);
  WriteRssrk(7, 0xD2E8684B);
  WriteRssrk(8, 0xECEF8417);
  WriteRssrk(9, 0xE56C84D5);

  // Fill in RSS redirection table (128 entries), sets which core the lowest 7 bits of hashed output goes to
  // hacky atm

  
  for (auto i = 0; i < 32; i += 4) {
    
    // hack for netpipe
    if(ncore == 1) {
      ebbrt::kprintf_force("+++ all interrupts firing on Core 0\n");
      WriteReta(i, 0x0000000);
      WriteReta(i+1, 0x0000000);
      WriteReta(i+2, 0x0000000);
      WriteReta(i+3, 0x0000000);
    } else {
      ebbrt::kprintf_force("+++ all interrupts firing on Core 1\n");
      WriteReta(i, 0x1010101);
      WriteReta(i+1, 0x1010101);
      WriteReta(i+2, 0x1010101);
      WriteReta(i+3, 0x1010101);
    }
    
    // all route to core 0
    /*if(ncore == 1) {
      WriteReta(i, 0x0000000);
      WriteReta(i+1, 0x0000000);
      WriteReta(i+2, 0x0000000);
      WriteReta(i+3, 0x0000000);
    } else if(ncore == 2) {
    WriteReta(i, 0x1010100);
      WriteReta(i+1, 0x1010100);
      WriteReta(i+2, 0x1010100);
      WriteReta(i+3, 0x1010100);
    } else if(ncore == 4) {
      WriteReta(i, 0x3020100);
      WriteReta(i+1, 0x3020100);
      WriteReta(i+2, 0x3020100);
      WriteReta(i+3, 0x3020100);
    } else if(ncore == 6) {
      WriteReta(i, 0x03020100);
      WriteReta(i+1, 0x01000504);
      WriteReta(i+2, 0x05040302);
      WriteReta(i+3, 0x04030201);
    } else if(ncore == 8) {
      WriteReta(i, 0x3020100);
      WriteReta(i+1, 0x7060504);
      WriteReta(i+2, 0x3020100);
      WriteReta(i+3, 0x7060504);
    } else if(ncore == 16){
      WriteReta(i, 0x03020100);
      WriteReta(i+1, 0x07060504); 
      WriteReta(i+2, 0x0B0A0908);
      WriteReta(i+3, 0x0F0E0D0C);
    } else {
      ebbrt::kabort("%s: Can only redirect interrupts to 16 cores\n", __FUNCTION__);
      }*/
  }
  
  for (auto i = 0; i < 128; i++) {
    WriteFtqf(i, 0x0);
    WriteSaqf(i, 0x0);
    WriteDaqf(i, 0x0);
    WriteSdpqf(i, 0x0);
  }

  // TODO SYNQF
  // TODO ETQF
  // TODO ETQS

  // Make sure RX CRC strip enabled in HLREG0 and RDRXCTL
  WriteRdrxctlRSCFRSTSIZE(~(0x1F << 17));  // s/w set to 0
  WriteRdrxctl(0x1 << 1);  // CRCStrip
  //WriteHlreg0(0x1 << 1);  // CRCStrip
  WriteHlreg0(0x2FFF);  // CRCStrip, Enable Jumbo Packets - Linux Default
  WriteRdrxctl(0x1 << 25);  // RSCACKC s/w set to 1
  WriteRdrxctl(0x1 << 26);  // FCOE_WRFIX s/w set to 1

  // Disable RSC for ACK Packets, disables the coalescing of TCP packets without TCP payload.
  // This bit should be set if performance problems are found.
  WriteRscdbu(0xa0);
  
  /***** END RX FILTER *****/

  // Configure buffers etc. according to specification
  // Section 4.6.11.3.4 (no DCB, no virtualization)

  /* Transmit Init: Set RTTDCS.ARBDIS to 1b.
   * Program DTXMXSZRQ, TXPBSIZE, TXPBTHRESH, MTQC, and MNGTXMAP, according
   * to the DCB and virtualization modes (see Section 4.6.11.3).
   * Clear RTTDCS.ARBDIS to 0b.
   */
  //WriteRttdcs(0x1 << 6);
  WriteRttdcs(0xC00040);
  WriteDtxmxszrq(MAX_BYTES_NUM_REQ);
  WriteTxpbsize(0, 0xA0 << 10);
  WriteTxpbThresh(0, 0xA0);
  for (auto i = 1; i < 8; i++) {
    WriteTxpbsize(i, 0x0);
    WriteTxpbThresh(i, 0x0);
  }
  WriteMtqc(0x0);
  WriteMngtxmap(0x0);
  WriteRttdcs(0xC00000);
  //WriteRttdcsArbdisEn(~(0x1 << 6));

  /* Receive Init: Program RXPBSIZE, MRQC, PFQDE, RTRUP2TC, MFLCN.RPFCE,
   * and MFLCN.RFCE according to the DCB and virtualization modes
   */
  WriteRxpbsize(0, 0x200 << 10);
  for (auto i = 1; i < 8; i++) {
    WriteRxpbsize(i, 0x0);
  }
  WriteMrqc(0x330001);

  WritePfqde(0x0);
  WriteRtrup2tc(0x0);
  WriteMflcn(0x0 << 2);
  WriteMflcn(0x1 << 3);
  // end DCB off, VT off

  // MAXFRS
  WriteMaxfrs(1518 << 16);
  //WriteMaxfrs(4096 << 16);
  
  // disable relaxed ordering
  /*for (auto i = 0; i < 128; i++) {
    WriteDcaTxctrlTxdescWbro(i, ~(0x1 << 11));  // Txdesc_Wbro

    if (i < 64) {
      WriteDcaRxctrl_1(
          i, ~(0x1 << 15));  // Rx split header relax order enable, bit 15
      WriteDcaRxctrl_1(
          i, ~(0x1 << 13));  // Rx data Write Relax Order Enable, bit 13
    } else {
      WriteDcaRxctrl_2(
          i - 64, ~(0x1 << 15));  // Rx split header relax order enable, bit 15
      WriteDcaRxctrl_2(
          i - 64, ~(0x1 << 13));  // Rx data Write Relax Order Enable, bit 13
    }
    }*/

#ifdef DCA_ENABLE
  // DCA_MODE = DCA 1.0
  WriteDcaCtrl(0x1 << 1);
  ebbrt::kprintf_force("+++ DCA Enabled\n");
#endif
}

void ebbrt::IxgbeDriver::FinishSetup() {
  // No snoop disable from FreeBSD ??
  WriteCtrlExt(0x10000);
  //WriteCtrlExt(0x1 << 16);  // NS_DIS
  /*for (size_t i = 0; i < Cpu::Count(); i++) {
    WriteDcaRxctrlClear(i, ~(0x1 << 12));  // clear bit 12
    }*/
  WriteEims(0xFFFF);
}

// initializes per core rx/tx queues and interrupts
void ebbrt::IxgbeDriver::SetupMultiQueue(uint32_t i) {
  if (!rcv_vector) {
    rcv_vector =
        event_manager->AllocateVector([this]() { ebb_->ReceivePoll(); });
  }

  // allocate memory for descriptor rings
  ixgmq[i].reset(new e10Kq(i, Cpu::GetMyNode()));

  // not going to set up receive descripts greater than 63
  ebbrt::kbugon(i >= 64, "can't set up descriptors greater than 63\n");

  // update register RDBAL, RDBAH with receive descriptor base address
  WriteRdbal_1(i, ixgmq[i]->rxaddr_ & 0xFFFFFFFF);
  WriteRdbah_1(i, (ixgmq[i]->rxaddr_ >> 32) & 0xFFFFFFFF);

  // set to number of bytes allocated for receive descriptor ring
  WriteRdlen_1(i, ixgmq[i]->rx_size_bytes_);

  // program srrctl register
  //WriteSrrctl_1(i, 0x2000402); // 3KB
  //WriteSrrctl_1(i, 0x2000410); // 16KB
  WriteSrrctl_1(i, (0x1 << 25) | (0x4 << 8) | (IxgbeDriver::RXBUFSZ / 1024));  // desctype adv 001b, BSIZEHEADER = 0x7 * 64B, BSIZEPACKET= 0x4 * 1 KB
  //WriteSrrctl_1(i, (0x1 << 25) | (0x4 << 8) | (3072 / 1024));
  
  /*WriteSrrctlZero(i);
    WriteSrrctl_1(i, RXBUFSZ / 1024);  // bsizepacket
  WriteSrrctl_1(i, (128 / 64) << 8);  // bsizeheader

// TODO headsplit adv
#ifdef RSC_EN
  WriteSrrctl_1(i, 0x1 << 25);  // desctype adv
#endif
#else
  // legacy is default??
  WriteSrrctl_1(i, ~(0x7 << 25));  // desctype legacy
  #endif

  WriteSrrctl_1(i, 0x1 << 28);  // Drop_En*/

#ifdef RSC_EN
  // RSC set up
  /****
       Maximum descriptors per Large receive as follow:
       00b = Maximum of 1 descriptor per large receive.
       01b = Maximum of 4 descriptors per large receive.
       10b = Maximum of 8 descriptors per large receive.
       11b = Maximum of 16 descriptors per large receive.

       (3:2) MAXDESC * SRRCTL.BSIZEPKT must not exceed 64 KB minus one, which is the
       maximum total length in the IP header and must be larger than the expected
       received MSS
  *****/
  
  WriteRscctl(i, 0x1 | (0x01 << 2));  // RSCEN=1, MAXDESC=  (0x1) * SRRCTL.BSIZEPACKET < 64KB
  WritePsrtype(i, 0x1 << 4); // 4.6.7.2.2 - PSR_type4 in PSRTYPE[n] should be set
#endif

  // In NON-IOV, only psrtype[0] is used
  //if (i == 0) {
  //  WritePsrtypeZero(0x1330);
  // }
    
  // Set head and tail pointers
  WriteRdt_1(i, 0x0);
  WriteRdh_1(i, 0x0);

  // Set Enable bit in receive queue
  WriteRxdctl_1_enable(i, 0x1 << 25);
  // TODO: Timeout
  while (ReadRxdctl_1_enable(i) == 0)
    ;

  // setup RX interrupts for queue i
  dev_.SetMsixEntry(i, rcv_vector, ebbrt::Cpu::GetByIndex(i)->apic_id());

  ebbrt::kprintf("Core %d: BSIZEPACKET=%d NTXDESCS=%d NRXDESCS=%d ITR_INTERVAL=%d us\n", i, RXBUFSZ, NTXDESCS, NRXDESCS, (int) (IxgbeDriver::ITR_INTERVAL * 2));
  
  //ebbrt::kprintf("Core %d: BSIZEPACKET=%d bytes NTXDESCS=%d NRXDESCS=%d ITR_INTERVAL=%dus RCV_VECTOR=%d APIC_ID=%d \n", i, RXBUFSZ, NTXDESCS, NRXDESCS, (int) (IxgbeDriver::ITR_INTERVAL * 2), (int)rcv_vector, (int)(ebbrt::Cpu::GetByIndex(i)->apic_id()));

  // don't set up interrupts for tx since we have head writeback??
  auto qn = i / 2;  // put into correct IVAR
  if ((i % 2) == 0) {  // check if 2xN or 2xN + 1
    WriteIvarAlloc0(qn, i | 0x1 << 7);  // rx interrupt allocation corresponds to index i *
                             // 2 in MSI-X table
    //WriteIvarAllocval0(qn, 0x1 << 7);
    WriteIvarAlloc0(qn, i << 8 | 0x1 << 15);
    //WriteIvarAllocval0(qn, 0x1 << 15);
  } else {
    WriteIvarAlloc2(qn, i << 16 | 0x1 << 23);
    //WriteIvarAllocval2(qn, 0x1 << 23);
    WriteIvarAlloc2(qn, i << 24 | 0x1 << 31);
    //WriteIvarAllocval2(qn, 0x1 << 31);
  }

  // must be greater than rsc delay  
  WriteEitr(i, (IxgbeDriver::ITR_INTERVAL << 3) | IXGBE_EITR_CNT_WDIS);
  // WriteEitr(i, 0x80 << 3); // 7 * 2us = 14 us
  //WriteEitr(i, (32 << 3) | IXGBE_EITR_CNT_WDIS);

  // 7.3.1.4 - Note that there are no EIAC(1)...EIAC(2) registers.
  // The hardware setting for interrupts 16...63 is always auto clear.
  if (i < 16) {
    // enable auto clear
    WriteEiac(0x1 << i);
  }

  // enable interrupt
  WriteEimsn(i / 32, (0x1 << (i % 32)));

  // make sure interupt is cleared
  if (i < 16) {
    WriteEicr(0x1 << i);
  }

  // Enable RX
  // disable RX_DIS
  //WriteSecrxctrl_Rx_Dis(0x1 << 1);
  WriteSecrxctrl_Rx_Dis(0x3);
  // TODO Timeout
  while (ReadSecrxstat_Sr_Rdy() == 0)
    ;
  WriteRxctrl(0x1);
  // enable RX_DIS, disable aes encryption offload, power savings
  WriteSecrxctrl_Rx_Dis(0x1);

  // add buffer to each descriptor
  for (size_t j = 0; j < NRXDESCS-1; j++) {
    auto rxphys =
        reinterpret_cast<uint64_t>((ixgmq[i]->circ_buffer_[j])->MutData());
    auto tail = ixgmq[i]->rx_tail_;

// update buffer address for descriptor
/*#ifdef RSC_EN 
    rdesc_adv_rf_t* tmp;
    tmp = reinterpret_cast<rdesc_adv_rf_t*>(&(ixgmq[i]->rx_ring_[tail]));

    tmp->packet_buffer = rxphys;
    // TODO only use this if enabling header splitting?
    tmp->header_buffer = 0;
#else*/
    ixgmq[i]->rx_ring_[tail].buffer_address = rxphys;
//#endif
    /*if(i == 0) {
      ebbrt::kprintf("rx_ring_[tail=%u].buffer_address = 0x%X\n", tail, rxphys);
      }*/

    ixgmq[i]->rx_tail_ = (tail + 1) % ixgmq[i]->rx_size_;
  }

  auto rxphys =
    reinterpret_cast<uint64_t>((ixgmq[i]->circ_buffer_[NRXDESCS-1])->MutData());
  ixgmq[i]->rx_ring_[ixgmq[i]->rx_tail_].buffer_address = rxphys;

  asm volatile("sfence" ::: "memory");
  // bump tail pts via register rdt to enable descriptor fetching by setting to
  // length of ring minus one
  WriteRdt_1(i, ixgmq[i]->rx_tail_);

#ifdef DCA_ENABLE
  auto myapic = ebbrt::Cpu::GetByIndex(i)->apic_id();

  WriteDcaRxctrl(i, 0x1 << 5);  // Descriptor DCA EN
  WriteDcaRxctrl(i, 0x1 << 6);  // Rx Header DCA EN
  WriteDcaRxctrl(i, 0x1 << 7);  // Payload DCA EN

  WriteDcaRxctrl(i, myapic << 24);  // CPUID = apic id

  WriteDcaTxctrl(i, 0x1 << 5);  // DCA Enable
  WriteDcaTxctrl(i, myapic << 24);  // CPUID = apic id
//#else
//  ReadDcaTxctrl(i);
//  ReadDcaRxctrl(i);
#endif

  // program base address registers
  WriteTdbal(i, ixgmq[i]->txaddr_ & 0xFFFFFFFF);
  WriteTdbah(i, (ixgmq[i]->txaddr_ >> 32) & 0xFFFFFFFF);

  // length must also be 128 byte aligned
  WriteTdlen(i, ixgmq[i]->tx_size_bytes_);

#ifdef TX_HEAD_WB
  ebbrt::kprintf_force("TX_HEAD_WB Enabled\n");
  WriteTdwbal(i, (ixgmq[i]->txhwbaddr_ & 0xFFFFFFFF) | 0x1);
  WriteTdwbah(i, (ixgmq[i]->txhwbaddr_ >> 32) & 0xFFFFFFFF);
#endif

  // enable transmit path: This step should be executed oly for the first enabled transmit queue and does
  // not need to be repeated for any following queues.
  if(i == 0) {
    WriteDmatxctl_te(0x1);
  }

  WriteTdh(i, 0x0);
    
  // transmit queue enable - PTHRESH=32 HTHRESH=1 WTHRESH=1 
  WriteTxdctl(i, 0x2010120);
  //WriteTxdctl(i, 0x2000000);
  
  // poll until set, TODO: Timeout
  while (ReadTxdctl_enable(i) == 0) {
    ebbrt::clock::SleepMilli(1);
  }

  WriteTdt(i, 0x0);
  ixgmq[i]->tx_tail_=0;

  // TODO: set up dca txctrl FreeBSD?
  // clear TXdescWBROen
  //WriteDcaTxctrlTxdescWbro(i, ~(0x1 << 11));
}

// Packet receive interrupt handler
void ebbrt::IxgbeDriverRep::ReceivePoll() {
  uint32_t  plen, i, icnt;
  uint64_t rxflag;
  rdesc_adv_wb_t* rx_desc;  
  uint32_t mcore = static_cast<uint32_t>(Cpu::GetMine());
  struct IxgbeLog *il;
  union IxgbeLogEntry *ile;
  uint64_t now = 0, last = 0;
  uint64_t cjoules, cins, ccyc, cllc;
  
  if(ixgmq_.collect_stats) {
    il = &ixgbe_logs[mcore];
    icnt = il->itr_cnt;
    
    if (icnt < IXGBE_LOG_SIZE) {
      ile = &il->log[icnt];
      
      //get current tsc and store it
      now = ebbrt::rdtsc();
      __builtin_ia32_movnti64(&ile->Fields.tsc, now);

      // get last tsc
      last = il->itr_joules_last_tsc;

      // ~ 1 ms has passed
      if ((now - last) > TSC_KHZ) { 	
	cjoules = ixgmq_.powerMeter.ReadMsr();
	cins = ixgmq_.perfInst.Read();
	ccyc = ixgmq_.perfCycles.Read();
	cllc = ixgmq_.perfLLC_miss.Read();

	__builtin_ia32_movnti64(&ile->Fields.joules, cjoules);
	__builtin_ia32_movnti64(&ile->Fields.ninstructions, cins);
	__builtin_ia32_movnti64(&ile->Fields.ncycles, ccyc);
	__builtin_ia32_movnti64(&ile->Fields.nllc_miss, cllc);

	il->itr_joules_last_tsc = now;
      }

      __builtin_ia32_movnti(&(ile->Fields.rx_desc), ixgmq_.stat_num_rx_desc);
      __builtin_ia32_movnti(&(ile->Fields.rx_bytes), ixgmq_.stat_num_rx_bytes);
      __builtin_ia32_movnti(&(ile->Fields.tx_desc), ixgmq_.stat_num_tx_desc);
      __builtin_ia32_movnti(&(ile->Fields.tx_bytes), ixgmq_.stat_num_tx_bytes);
      
      ixgmq_.stat_num_rx_bytes = 0;
      ixgmq_.stat_num_rx_desc = 0;
      ixgmq_.stat_num_tx_bytes = 0;
      ixgmq_.stat_num_tx_desc = 0;
      il->itr_cnt++;
    }
  }
  
/* if(ixgmq_.collect_stats) {
    // log itr
    //auto d = ebbrt::clock::Wall::Now().time_since_epoch();
    //uint64_t now = static_cast<uint64_t>(
    //std::chrono::duration_cast<std::chrono::microseconds>(d).count());
    
    //uint64_t last = ixgmq_.itr_joules_last_ts;
    auto now = ebbrt::clock::TscToNano(ebbrt::rdtsc());
    auto last = ixgmq_.itr_joules_last_ts;
    
    //if((now - last) > 1000) { //1 ms has passed
    if((now - last).count() > 1000000) { //1 ms has passed
      // if perf counters have already started
      if(ixgmq_.perf_started) {
	// stop
	ixgmq_.perfCycles.Stop();
	ixgmq_.perfInst.Stop();
	ixgmq_.perfLLC_miss.Stop();
      
	// read counters
	ixgmq_.itr_stats.push_back({ixgmq_.powerMeter.ReadMsr(), now, ixgmq_.stat_num_rx_desc, ixgmq_.stat_num_rx_bytes, ixgmq_.stat_num_tx_desc, ixgmq_.stat_num_tx_bytes, ixgmq_.perfInst.Read(), ixgmq_.perfCycles.Read(), ixgmq_.perfLLC_miss.Read()});
      
	// clear
	ixgmq_.perfCycles.Clear();
	ixgmq_.perfInst.Clear();
	ixgmq_.perfLLC_miss.Clear();
      }
    
      // start counters
      ixgmq_.perf_started = 1;
      ixgmq_.perfCycles.Start();
      ixgmq_.perfInst.Start();
      ixgmq_.perfLLC_miss.Start();
    
      ixgmq_.itr_joules_last_ts = now;    
    } else {
      ixgmq_.itr_stats.push_back({0, now, ixgmq_.stat_num_rx_desc, ixgmq_.stat_num_rx_bytes, ixgmq_.stat_num_tx_desc, ixgmq_.stat_num_tx_bytes, 0, 0, 0});
    }
  
    ixgmq_.stat_num_rx_bytes = 0;
    ixgmq_.stat_num_rx_desc = 0;
    ixgmq_.stat_num_tx_bytes = 0;
    ixgmq_.stat_num_tx_desc = 0;  
  }
*/
  
  i = 0;
  //while(i < 64) {
  while(i < IxgbeDriver::NRXDESCS) {
    rxflag = 0x0;
    rx_desc = reinterpret_cast<rdesc_adv_wb_t*>(&(ixgmq_.rx_ring_[ixgmq_.rx_head_]));

    // Linux's ixgbe:
     // This memory barrier is needed to keep us from reading                      
     // any other fields out of the rx_desc until we know the                      
     // descriptor has been written back                                           
     //
    asm volatile("" ::: "memory");
  
    // if no rx packets ready to be processed
    if (!(rx_desc->dd)) {
      return;
    }    

    // return buffers to hardware
    if(ixgmq_.cleaned_count > IXGBE_RX_BUFFER_WRITE) {
      //ebbrt::kprintf_force("START c=%u cleaned_count=%u head=%u tail=%u ", mcore, ixgmq_.cleaned_count,
      //		     ixgmq_.rx_head_, ixgmq_.rx_tail_);
      
      while(ixgmq_.cleaned_count) {
	// reset buffer
	ixgmq_.rx_ring_[ixgmq_.rx_tail_].raw[0] = 0x0;
	ixgmq_.rx_ring_[ixgmq_.rx_tail_].raw[1] = 0x0;

	// allocate new rx buffer
	ixgmq_.circ_buffer_[ixgmq_.rx_tail_] = std::move(MakeUniqueIOBuf(IxgbeDriver::RXBUFSZ));
	auto rxphys =
	  reinterpret_cast<uint64_t>((ixgmq_.circ_buffer_[ixgmq_.rx_tail_])->MutData());
	// update descriptor with new buffer adder
	ixgmq_.rx_ring_[ixgmq_.rx_tail_].buffer_address = rxphys;      
	
	ixgmq_.rx_tail_ = (ixgmq_.rx_tail_ + 1) % ixgmq_.rx_size_;
	ixgmq_.cleaned_count --;
      }      
      
      ixgmq_.cleaned_count = 0;
      
      // Force memory writes to complete before letting h/w
      //know there are new descriptors to fetch.  (Only
      // applicable for weak-ordered memory model archs,
      // such as IA-64).
      //
      wmb();
      WriteRdt_1(mcore, ixgmq_.rx_tail_);
      
      //ebbrt::kprintf_force("END c=%u cleaned_count=%u head=%u tail=%u\n", mcore, ixgmq_.cleaned_count,
//			   ixgmq_.rx_head_, ixgmq_.rx_tail_);
    }
      
    // handle a single receive
    if(rx_desc->eop)
    {
      ixgmq_.rx_desc_counts[1] ++;
      
      //if (cleaned_count >= IXGBE_RX_BUFFER_WRITE)    
      plen = rx_desc->pkt_len;
      if(!plen) return; // Linux's ixgbe driver checks this case
      //ixgmq_.stat_num_rx_bytes += plen;
      
      // TCP/UDP checksum
      if (rx_desc->l4i) {
	rxflag |= RXFLAG_L4CS;
	if (!(rx_desc->l4e)) {
	  rxflag |= RXFLAG_L4CS_VALID;
	}
      }
      // Ipv4 checksum
      if (rx_desc->ipcs) {
	rxflag |= RXFLAG_IPCS;
	if (!(rx_desc->ipe)) {
	  rxflag |= RXFLAG_IPCS_VALID;
	}
      }

      // setup rx buffers
      ixgmq_.circ_buffer_[ixgmq_.rx_head_]->SetLength(plen);
      auto b = std::move(ixgmq_.circ_buffer_[ixgmq_.rx_head_]);
      ixgmq_.cleaned_count ++;
      ixgmq_.stat_num_rx_desc ++;
      i ++;
      
      //ebbrt::kprintf_force("ReceivePoll() core=%u, head=%u tail=%u plen=%u\n",
//			 mcore, ixgmq_.rx_head_, ixgmq_.rx_tail_, plen);
      // update next rx descriptor to process
      //ntc = ixgmq_.rx_head_ + 1;
      //ntc = (ntc < ixgmq_.rx_size_) ? ntc : 0;
      //ixgmq_.rx_head_ = ntc;
      ixgmq_.rx_head_ = (ixgmq_.rx_head_ + 1) % ixgmq_.rx_size_;

      //ebbrt::kprintf_force("before rxd= %u rxb = %u\n", rxd, rxb);      
      // TODO add _mm_prefetch from Linux??
      ixgmq_.total_rx_bytes += b->ComputeChainDataLength();
      ixgmq_.stat_num_rx_bytes += b->ComputeChainDataLength();
      root_.itf_.Receive(std::move(b), rxflag);
      //ebbrt::kprintf_force("after rxd= %u rxb = %u\n", rxd, rxb);
    }
    else
    {
#ifndef RSC_EN
      ebbrt::kprintf_force("RSC packet in non-RSC mode\n");
      return;
#endif
      
      uint32_t rsc_count = 0;
      
      //RSC FIRST PACKET
      plen = rx_desc->pkt_len;    
      if(!plen) {
	ebbrt::kabort("+++ RSC first packet Abort: core %u pkt_len == 0.\n", mcore);
	return;
      }
      //ixgmq_.stat_num_rx_bytes += plen;

      
      //ebbrt::kprintf("\nRSC desc=%d next_desc=%d len=%d eop=%d\n", ixgmq_.rx_head_, rx_desc->next_descriptor_ptr, plen, rx_desc->eop);
      
      // setup rx buffers
      ixgmq_.circ_buffer_[ixgmq_.rx_head_]->SetLength(plen);
      auto b = std::move(ixgmq_.circ_buffer_[ixgmq_.rx_head_]);
      ixgmq_.cleaned_count ++;
      ixgmq_.stat_num_rx_desc ++;
      i ++;
      ixgmq_.rx_head_ = (ixgmq_.rx_head_ + 1) % ixgmq_.rx_size_;
      rsc_count ++;
      
      while(true) {
	rx_desc = reinterpret_cast<rdesc_adv_wb_t*>(&(ixgmq_.rx_ring_[ixgmq_.rx_head_]));
      retry:
      // Linux's ixgbe:
	 // This memory barrier is needed to keep us from reading                      
	 // any other fields out of the rx_desc until we know the                      
	 // descriptor has been written back                                           
	 //
	asm volatile("" ::: "memory");
	
	// if no rx packets ready
	if (!(rx_desc->dd)) {
	  goto retry;
	  //ebbrt::kprintf_force("**** RSC Abort: core %u rx_desc->dd == 0\n", mcore);
	  //return;
	  //mb();
	  //rmb();
	  //wmb();
	}

	plen = rx_desc->pkt_len;    
	if(!plen) {
	  ebbrt::kabort("+++ RSC middle packetsAbort: core %u pkt_len == 0.\n", mcore);
	  return;
	}
	//ixgmq_.stat_num_rx_bytes += plen;

	
	// setup rx buffers
	ixgmq_.circ_buffer_[ixgmq_.rx_head_]->SetLength(plen);
        b->PrependChain(std::move(ixgmq_.circ_buffer_[ixgmq_.rx_head_]));
	ixgmq_.cleaned_count ++;
	ixgmq_.stat_num_rx_desc ++;
	i ++;	

	//ebbrt::kprintf("\nRSC desc=%d next_desc=%d len=%d eop=%d\n", ixgmq_.rx_head_, rx_desc->next_descriptor_ptr, plen, rx_desc->eop);
	
	ixgmq_.rx_head_ = (ixgmq_.rx_head_ + 1) % ixgmq_.rx_size_;
	
	rsc_count ++;
	if(rx_desc->eop) {
	  rxflag = 0;
	  // TCP/UDP checksum
	  if (rx_desc->l4i) {
	    rxflag |= RXFLAG_L4CS;
	    if (!(rx_desc->l4e)) {
	      rxflag |= RXFLAG_L4CS_VALID;
	    }
	  }
	  // Ipv4 checksum
	  if (rx_desc->ipcs) {
	    rxflag |= RXFLAG_IPCS;
	    if (!(rx_desc->ipe)) {
	      rxflag |= RXFLAG_IPCS_VALID;
	    }
	  }
	  
	  ixgmq_.rx_desc_counts[rsc_count] ++;
	  //if(b->ComputeChainDataLength() > 256) {
	  //  auto p1 = reinterpret_cast<uint8_t*>(b->MutData());
	  //  for (int i = 0; i < 248; i+=8) {
	  //   ebbrt::kprintf_force("%02X%02X%02X%02X%02X%02X%02X%02X\n", p1[i], p1[i+1], p1[i+2], p1[i+3], p1[i+4], p1[i+5], p1[i+6], p1[i+7]);
	  //  }
	  //}
	  //ebbrt::kprintf("RSC len=%u rsc_count=%d\n", b->ComputeChainDataLength(), rsc_count);
	  break;
	}
      }
      ixgmq_.total_rx_bytes += b->ComputeChainDataLength();
      ixgmq_.stat_num_rx_bytes += b->ComputeChainDataLength();
      root_.itf_.Receive(std::move(b), rxflag);
    }    
  }
}

/*void ebbrt::IxgbeDriverRep::ReceivePoll() {
  uint32_t len;
  uint64_t bAddr;
  uint64_t rxflag;
  bool process_rsc;
  uint32_t count;
  uint32_t rnt;
  uint32_t rxhead;
  process_rsc = false;
  rxflag = 0;
  count = 0;
  rnt = 0;
  uint32_t mcore = static_cast<uint32_t>(Cpu::GetMine());
#ifdef STATS_EN
  ixgmq_.stat_num_recv ++;
#endif
  
  // while there are still packets received
  while (GetRxBuf(&len, &bAddr, &rxflag, &process_rsc, &rnt, &rxhead) == 1) {
    // hit last rsc context, start to process all buffers
    if (process_rsc) {
    } else {
      count ++;

      //ebbrt::kprintf("Core: %d ReceivePoll() len=%d rxhead=%d\n", mcore, len, rxhead);

#ifdef STATS_EN
      ixgmq_.stat_num_rx_bytes += len;
#endif
      
      ixgmq_.circ_buffer_[rxhead]->SetLength(len);
      auto b = std::move(ixgmq_.circ_buffer_[rxhead]);

      // bump tail ptr
      ixgmq_.rx_tail_ = (ixgmq_.rx_tail_ + 1) % ixgmq_.rx_size_;
      root_.itf_.Receive(std::move(b), rxflag);

      // reset buffer
      ixgmq_.rx_ring_[rxhead].raw[0] = 0;
      ixgmq_.rx_ring_[rxhead].raw[1] = 0;
      // allocate new rx buffer
      ixgmq_.circ_buffer_[rxhead] = std::move(MakeUniqueIOBuf(IxgbeDriver::RXBUFSZ));
      auto rxphys =
        reinterpret_cast<uint64_t>((ixgmq_.circ_buffer_[rxhead])->MutData());
      // update buffer with new adder
      ixgmq_.rx_ring_[rxhead].buffer_address = rxphys;

      wmb();
      WriteRdt_1(mcore, ixgmq_.rx_tail_);      
    }
  }
  }*/

ebbrt::IxgbeDriverRep::IxgbeDriverRep(const IxgbeDriver& root)
    : root_(root), ixgq_(root_.GetQueue()),
      ixgmq_(root.GetMultiQueue(Cpu::GetMine())),
      receive_callback_([this]() { ReceivePoll(); }) {
  //this->ReceivePoll();
  ixgmq_.perfCycles = ebbrt::perf::PerfCounter(ebbrt::perf::PerfEvent::cycles);
  ixgmq_.perfInst = ebbrt::perf::PerfCounter(ebbrt::perf::PerfEvent::instructions);
  ixgmq_.perfLLC_miss = ebbrt::perf::PerfCounter(ebbrt::perf::PerfEvent::llc_misses);
  
  ixgmq_.powerMeter = ebbrt::rapl::RaplCounter();
}

void ebbrt::IxgbeDriverRep::IxgbeDriverRep::StartTimer() {
  auto timeout = std::chrono::seconds(1);
  timer->Start(*this, timeout, true);
}

void ebbrt::IxgbeDriverRep::IxgbeDriverRep::StopTimer() {
  timer->Stop(*this);
}
  
void ebbrt::IxgbeDriverRep::IxgbeDriverRep::Fire() {
  uint32_t mcore = static_cast<uint32_t>(Cpu::GetMine());

  ixgmq_.perfCycles.Stop();
  ixgmq_.perfInst.Stop();
  ixgmq_.perfLLC_miss.Stop();  
  //if(mcore == 0 || mcore == 1) {
  ixgmq_.powerMeter.Stop();
    //}
  // accumulate counters
  ixgmq_.totalCycles += static_cast<uint64_t>(ixgmq_.perfCycles.Read());
  ixgmq_.totalIns += static_cast<uint64_t>(ixgmq_.perfInst.Read());
  ixgmq_.totalLLCmisses += static_cast<uint64_t>(ixgmq_.perfLLC_miss.Read());
  //if(mcore == 0 || mcore == 1) {
  ixgmq_.totalNrg += ixgmq_.powerMeter.Read();
  //ebbrt::kprintf_force("Core %u: Fire() cycles=%llu ins=%llu llc=%llu energy=%.2lfJ\n", mcore, ixgmq_.totalCycles, ixgmq_.totalIns, ixgmq_.totalLLCmisses, ixgmq_.totalNrg);
  ebbrt::kprintf_force("Core %u: Fire() energy=%.4lf J\n", mcore, ixgmq_.totalNrg);
  ixgmq_.totalNrg = 0;
    //}
  
  ixgmq_.perfCycles.Clear();
  ixgmq_.perfInst.Clear();
  ixgmq_.perfLLC_miss.Clear();
  
  ixgmq_.perfCycles.Start();
  ixgmq_.perfInst.Start();
  ixgmq_.perfLLC_miss.Start();
  //if(mcore == 0 || mcore == 1) {
  ixgmq_.powerMeter.Start();
    //}
  ixgmq_.fireCount += 1;  
  //ebbrt::kprintf_force("Core %u: Fire() %llu\n", mcore, ixgmq_.fireCount);
}

uint16_t ebbrt::IxgbeDriverRep::ReadRdh_1(uint32_t n) {
  auto reg = root_.bar0_.Read32(0x01010 + 0x40 * n);
  return reg & 0xFFFF;
}
uint16_t ebbrt::IxgbeDriverRep::ReadRdt_1(uint32_t n) {
  auto reg = root_.bar0_.Read32(0x01018 + 0x40 * n);
  return reg & 0xFFFF;
}

void ebbrt::IxgbeDriverRep::WriteRdt_1(uint32_t n, uint32_t m) {
  root_.bar0_.Write32(0x01018 + 0x40 * n, m);
}

void ebbrt::IxgbeDriverRep::Run() {
  while (1) {
    ReceivePoll();
  }
}
void ebbrt::IxgbeDriverRep::WriteTdt_1(uint32_t n, uint32_t m) {
  root_.bar0_.Write32(0x06018 + 0x40 * n, m);
}

// 8.2.3.5.9 Extended Interrupt Mask Clear Registers — EIMC[n]
// (0x00AB0 + 4*(n-1), n=1...2; WO)
void ebbrt::IxgbeDriverRep::WriteEimcn(uint32_t n, uint32_t m) {
  auto reg = root_.bar0_.Read32(0x00AB0 + 4 * n);
  root_.bar0_.Write32(0x00AB0 + 4 * n, reg | m);
}

// 8.2.3.5.4 Extended Interrupt Mask Clear Register- EIMC (0x00888; WO)
void ebbrt::IxgbeDriverRep::WriteEimc(uint32_t m) { root_.bar0_.Write32(0x00888, m); }

// 8.2.3.5.3 Extended Interrupt Mask Set/Read Register- EIMS (0x00880; RWS)
void ebbrt::IxgbeDriverRep::WriteEims(uint32_t m) { root_.bar0_.Write32(0x00880, m); }

uint32_t ebbrt::IxgbeDriverRep::ReadTdh_1(uint32_t n) {
  auto reg = root_.bar0_.Read32(0x06010 + 0x40 * n);
  return reg & 0xFFFF;
}
uint32_t ebbrt::IxgbeDriverRep::ReadTdt_1(uint32_t n) {
  return root_.bar0_.Read32(0x06018 + 0x40 * n) & 0xFFFF;
}
