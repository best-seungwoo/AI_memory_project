#include "Controller.h"
#include "SALP.h"
#include "ALDRAM.h"
#include "TLDRAM.h"

using namespace ramulator;

namespace ramulator
{

static vector<int> get_offending_subarray(DRAM<SALP>* channel, vector<int> & addr_vec){
    int sa_id = 0;
    auto rank = channel->children[addr_vec[int(SALP::Level::Rank)]];
    auto bank = rank->children[addr_vec[int(SALP::Level::Bank)]];
    auto sa = bank->children[addr_vec[int(SALP::Level::SubArray)]];
    for (auto sa_other : bank->children)
        if (sa != sa_other && sa_other->state == SALP::State::Opened){
            sa_id = sa_other->id;
            break;
        }
    vector<int> offending = addr_vec;
    offending[int(SALP::Level::SubArray)] = sa_id;
    offending[int(SALP::Level::Row)] = -1;
    return offending;
}


template <>
vector<int> Controller<SALP>::get_addr_vec(SALP::Command cmd, list<Request>::iterator req){
    if (cmd == SALP::Command::PRE_OTHER)
        return get_offending_subarray(channel, req->addr_vec);
    else
        return req->addr_vec;
}


template <>
bool Controller<SALP>::is_ready(list<Request>::iterator req){
    SALP::Command cmd = get_first_cmd(req);
    if (cmd == SALP::Command::PRE_OTHER){

        vector<int> addr_vec = get_offending_subarray(channel, req->addr_vec);
        return channel->check(cmd, addr_vec.data(), clk);
    }
    else return channel->check(cmd, req->addr_vec.data(), clk);
}

template <>
void Controller<ALDRAM>::update_temp(ALDRAM::Temp current_temperature){
    channel->spec->aldram_timing(current_temperature);
}


template <>
void Controller<TLDRAM>::tick(){
    clk++;
    req_queue_length_sum += readq.size() + writeq.size();
    read_req_queue_length_sum += readq.size();
    write_req_queue_length_sum += writeq.size();

    /*** 1. Serve completed reads ***/
    if (pending.size()) {
        Request& req = pending[0];
        if (req.depart <= clk) {
          if (req.depart - req.arrive > 1) {
                  read_latency_sum += req.depart - req.arrive;
                  channel->update_serving_requests(
                      req.addr_vec.data(), -1, clk);
          }
            req.callback(req);
            pending.pop_front();
        }
    }

    /*** 2. Should we schedule refreshes? ***/
    refresh->tick_ref();

    /*** 3. Should we schedule writes? ***/
    if (!write_mode) {
        // yes -- write queue is almost full or read queue is empty
        if (writeq.size() >= int(0.8 * writeq.max) /*|| readq.size() == 0*/)
            write_mode = true;
    }
    else {
        // no -- write queue is almost empty and read queue is not empty
        if (writeq.size() <= int(0.2 * writeq.max) && readq.size() != 0)
            write_mode = false;
    }

    /*** 4. Find the best command to schedule, if any ***/
    Queue* queue = !write_mode ? &readq : &writeq;
    if (otherq.size())
        queue = &otherq;  // "other" requests are rare, so we give them precedence over reads/writes

    auto req = scheduler->get_head(queue->q);
    if (req == queue->q.end() || !is_ready(req)) {
        // we couldn't find a command to schedule -- let's try to be speculative
        auto cmd = TLDRAM::Command::PRE;
        vector<int> victim = rowpolicy->get_victim(cmd);
        if (!victim.empty()){
            issue_cmd(cmd, victim);
        }
        return;  // nothing more to be done this cycle
    }

    if (req->is_first_command) {
        int coreid = req->coreid;
        req->is_first_command = false;
        if (req->type == Request::Type::READ || req->type == Request::Type::WRITE) {
          channel->update_serving_requests(req->addr_vec.data(), 1, clk);
        }
        int tx = (channel->spec->prefetch_size * channel->spec->channel_width / 8);
        if (req->type == Request::Type::READ) {
            if (is_row_hit(req)) {
                ++read_row_hits[coreid];
                ++row_hits;
            } else if (is_row_open(req)) {
                ++read_row_conflicts[coreid];
                ++row_conflicts;
            } else {
                ++read_row_misses[coreid];
                ++row_misses;
            }
          read_transaction_bytes += tx;
        } else if (req->type == Request::Type::WRITE) {
          if (is_row_hit(req)) {
              ++write_row_hits[coreid];
              ++row_hits;
          } else if (is_row_open(req)) {
              ++write_row_conflicts[coreid];
              ++row_conflicts;
          } else {
              ++write_row_misses[coreid];
              ++row_misses;
          }
          write_transaction_bytes += tx;
        }
    }

    /*** 5. Change a read request to a migration request ***/
    if (req->type == Request::Type::READ) {
        req->type = Request::Type::EXTENSION;
    }

    // issue command on behalf of request
    auto cmd = get_first_cmd(req);
    issue_cmd(cmd, get_addr_vec(cmd, req));

    // check whether this is the last command (which finishes the request)
    if (cmd != channel->spec->translate[int(req->type)])
        return;

    // set a future completion time for read requests
    if (req->type == Request::Type::READ || req->type == Request::Type::EXTENSION) {
        req->depart = clk + channel->spec->read_latency;
        pending.push_back(*req);
    }
    if (req->type == Request::Type::WRITE) {
        channel->update_serving_requests(req->addr_vec.data(), -1, clk);
    }

    // remove request from queue
    queue->q.erase(req);
}

template <>
void Controller<DDR3>::tick()
{
        clk++;
        req_queue_length_sum += readq.size() + writeq.size() + pending.size() + rngq.size(); //seungwoo
        read_req_queue_length_sum += readq.size() + pending.size() + rngq.size(); //seungwoo
        write_req_queue_length_sum += writeq.size();
        idle_period_end = false;
        
        /*** 1. Serve completed reads ***/
        if (pending.size()) {
            Request& req = pending[0];
            if (req.depart <= clk) {
                if (req.depart - req.arrive > 1) { // this request really accessed a row
                  read_latency_sum += req.depart - req.arrive;
                  channel->update_serving_requests(
                      req.addr_vec.data(), -1, clk);
                }
                req.callback(req);
                pending.pop_front();
            }
        }

        /***seungwoo: 1.5. Serve completed RNG reads ***/
        if (rng_pending.size()) {
            Request& req = rng_pending[0];
            if (req.depart <= clk) {
                if (req.depart - req.arrive > 1) { // this request really accessed a row
                  //read_latency_sum += req.depart - req.arrive;
                  //channel->update_serving_requests(
                  //    req.addr_vec.data(), -1, clk);
                }
                req.callback(req);
                rng_pending.pop_front();
            }
        }

        /*** 2. Refresh scheduler ***/
        refresh->tick_ref();

        /*** 3. Should we schedule writes? ***/
        if (!write_mode) {
            // yes -- write queue is almost full or read queue is empty
            if (writeq.size() > int(wr_high_watermark * writeq.max) || readq.size() == 0){
                write_mode = true;
            }
        }
        else {
            // no -- write queue is almost empty and read queue is not empty
            if (writeq.size() < int(wr_low_watermark * writeq.max) && readq.size() != 0){
                write_mode = false;
            }
        }

        /*** 4. Find the best command to schedule, if any ***/

        // First check the actq (which has higher priority) to see if there
        // are requests available to service in this cycle
        Queue* queue = &actq;
        DDR3::Command cmd;
        if(RNG_mode){
            queue = &rngq;
            Request request(0x12312312, Request::Type::RNG);
            (queue->q).push_back(request);
        }
        auto req = scheduler->get_head(queue->q);
        //seungwoo: if RNG_mode, generate a random number first.
        std::cout<<"req_type: "<<((req->name_type).find(req->type))->second<<std::endl; //seungwoo: check request

        bool is_valid_req = (req != queue->q.end());

//std::cout<<"RNG는 요 앞에서 막힘"<<std::endl;
        int sat_counter;
        last_acc_addr = req->addr;
        map<long, int>::iterator it = idle_pred_table.find(last_acc_addr);
        if(it != idle_pred_table.end()){
            sat_counter = it->second;
        } //exist entry -> get 2-bit counter
        else{
            sat_counter = 1;
            idle_pred_table.insert({last_acc_addr, sat_counter});
        } //no entry in table -> initialize 2-bit counter as 01
        switch (sat_counter){
            case 0:
            case 1: break;
            case 2:
            case 3: RNG_mode = true;
            //seungwoo: set RNG_mode which means that
            //we should make random number in next tick.
        }
        //seungwoo: update last_accessed memory address
        //and lookup the idleness prediction table using the address.
        //If 2-bit counter is 0 or 1, insert an RNG request.

        if(is_valid_req) {
            idle_period_end = true;
            idle_period_len_buff = idle_period_len;
            idle_period_len = 0;
            //seungwoo: when a valid request is in
            //store the idle period and reset as zero
            cmd = get_first_cmd(req);
            is_valid_req = is_ready(cmd, req->addr_vec);
        }

        if (!is_valid_req) {
            /*original code: queue = !write_mode ? &readq : &writeq;*/
            //int priority_case = compare_rng_app_priority();
            //This should be done with OS support,
            //but I just replaced it as a random function.
            int priority_case = rand() % 3;
            if(write_mode){
                queue = &writeq;
            }
            else{   
                switch (priority_case){
                    case 0: queue = &rngq; break;
                    case 1: queue = &readq; break;
                    case 2: queue = &rngq; break;
                }
            } //seungwoo

            if (otherq.size()){
                queue = &otherq;  // "other" requests are rare, so we give them precedence over reads/writes
            }
            req = scheduler->get_head(queue->q);

            is_valid_req = (req != queue->q.end());

            if(is_valid_req){
                idle_period_end = true;
                idle_period_len_buff = idle_period_len;
                idle_period_len = 0;
                //seungwoo: when a valid request is in
                //store the idle period and reset as zero
                cmd = get_first_cmd(req);
                is_valid_req = is_ready(cmd, req->addr_vec);
            }
        }

        if (!is_valid_req) {
            idle_period_len++; //seungwoo
            // we couldn't find a command to schedule -- let's try to be speculative
            auto cmd = DDR3::Command::PRE;
            vector<int> victim = rowpolicy->get_victim(cmd);
            if (!victim.empty()){
                issue_cmd(cmd, victim);
            }
            return;  // nothing more to be done this cycle
        }

        if (req->is_first_command) {
            req->is_first_command = false;
            int coreid = req->coreid;
            if (req->type == Request::Type::READ || req->type == Request::Type::WRITE /*seungwoo*/ || req->type == Request::Type::RNG) {
              channel->update_serving_requests(req->addr_vec.data(), 1, clk);
            }
            int tx = (channel->spec->prefetch_size * channel->spec->channel_width / 8);
            if (req->type == Request::Type::READ) {
                if (is_row_hit(req)) {
                    ++read_row_hits[coreid];
                    ++row_hits;
                } else if (is_row_open(req)) {
                    ++read_row_conflicts[coreid];
                    ++row_conflicts;
                } else {
                    ++read_row_misses[coreid];
                    ++row_misses;
                }
              read_transaction_bytes += tx;
            } else if (req->type == Request::Type::WRITE) {
              if (is_row_hit(req)) {
                  ++write_row_hits[coreid];
                  ++row_hits;
              } else if (is_row_open(req)) {
                  ++write_row_conflicts[coreid];
                  ++row_conflicts;
              } else {
                  ++write_row_misses[coreid];
                  ++row_misses;
              }
              write_transaction_bytes += tx;
            }
        }

        std::cout<<"cmd: "<<channel->spec->command_name[(int)cmd]<<std::endl; //seungwoo: check cmd

        // issue command on behalf of request
        issue_cmd(cmd, get_addr_vec(cmd, req));

        // check whether this is the last command (which finishes the request)
        //if (cmd != channel->spec->translate[int(req->type)]){
        if (cmd != channel->spec->translate[int(req->type)]) {
            if(channel->spec->is_opening(cmd)) {
                // promote the request that caused issuing activation to actq
                actq.q.push_back(*req);
                queue->q.erase(req);
            }

            return;
        }


        // set a future completion time for read requests
        if (req->type == Request::Type::READ) {
            req->depart = clk + channel->spec->read_latency;
            pending.push_back(*req);
        }

        if (req->type == Request::Type::WRITE) {
            channel->update_serving_requests(req->addr_vec.data(), -1, clk);
            // req->callback(*req);
        }

        if (req->type == Request::Type::RNG) {
            RNB.push(rand());
            req->depart = clk + channel->spec->rng_read_latency;
            rng_pending.push_back(*req);
        } //seungwoo

        if(idle_period_end){
            if(idle_period_len_buff >= channel->spec->rng_read_latency){
                if(sat_counter >= 0 && sat_counter <= 2){
                    it->second = sat_counter + 1;
                }
            } //taken was the right prediction
            else{
                if(sat_counter >= 1 && sat_counter <= 3){
                    it->second = sat_counter - 1;
                }
            } //not taken was the right prediction
        }
        //seungwoo: update the saturating counter according to the result
        //whether the actual idle period was longer than the rng_read_latency.

        // remove request from queue
        queue->q.erase(req);
}

template<>
void Controller<TLDRAM>::cmd_issue_autoprecharge(typename TLDRAM::Command& cmd,
                                                    const vector<int>& addr_vec) {
    //TLDRAM currently does not have autoprecharge commands
    return;
}

} /* namespace ramulator */
