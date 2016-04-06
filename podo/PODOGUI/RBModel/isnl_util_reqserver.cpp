#include <isnl/util/reqserver.h>

namespace isnl{
struct ReqServerParamStruct{
	ReqServer* sv;
	PacketHandler* hd;
    SOCKET as;
};
void rsockListener(void *data){
	ReqServer * sv = (ReqServer*)data;
    SOCKET as;
    while(sv->state==1){
        as = accept(sv->ls, NULL, NULL);
        if (as != SOCKET_ERROR){
            ReqServerParamStruct *param = new ReqServerParamStruct;
            param->sv = sv;
            param->hd = (sv->handler ? sv->handler->initialize() : NULL);
            param->as = as;
            tthread::thread *th = new tthread::thread(rsockRequestHandler, param);
            sv->handle.push_back(th);
        }
    }
    // join all handle threads
    int n = sv->handle.size();
    printf("Lister finalize started\n");
    for(int i = 0; i < n; ++i){
        printf("Closing %d-th handle\n",i);
        sv->handle[i]->join();
        delete sv->handle[i];
    }
    printf("Lister finalize complete\n");
}
void rsockRequestHandler(void *data){
    // get parameters
    ReqServerParamStruct *param = (ReqServerParamStruct*)data;
    ReqServer* sv = param->sv;
    PacketHandler* handler = param->hd;
    MsgClientHandle ch(param->as);
    delete param;
    Packet req;       // request packet
    bool cont = true;

    while(cont && sv->state==1){
        try{
            ch.recv(req);
            (*handler)(ch, req);
        }catch(SocketFailException e){
            cont = false;
        }
    }
    handler->finalize();
}

}
