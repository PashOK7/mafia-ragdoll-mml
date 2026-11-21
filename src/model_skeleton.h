#pragma once
#include <vector>
#include "mathx.h"
#include "mod.h"

struct SkeletonTransform { V3 pos; M3 basis; };

// Keep the file's hierarchy and bind transforms. The model handle also owns
// geometry and is released before the caller retains its small skeleton.
inline bool ReadSkeleton(const char *file,std::vector<MmlModelNode> &nodes,
                         std::vector<SkeletonTransform> &world,std::string *error) {
    const auto *host=mod::H;
    auto *mod=mod::M;
    const int handle=host->model_open(mod,file);
    if (!handle) {
        *error=std::string("cannot read rest skeleton: ")+file;
        return false;
    }
    const int count=host->model_nodes(mod,handle,nullptr,0);
    if (count>0 && count<=4096) {
        nodes.resize(count);
        host->model_nodes(mod,handle,nodes.data(),count);
    }
    host->model_close(mod,handle);
    if (nodes.empty()) {
        *error="invalid rest skeleton node count";
        return false;
    }
    world.resize(count);
    for (int i=0;i<count;++i) {
        const auto &node=nodes[i];
        if (node.parent>=i || node.parent<-1) {
            *error="rest skeleton parent order is invalid";
            return false;
        }
        float matrix[16];
        QuatMatrix({node.rot[0],node.rot[1],node.rot[2],node.rot[3]},matrix);
        M3 basis=M3::Raw(matrix);
        for (int k=0;k<3;++k) basis.r[k]=basis.r[k]*node.scale[k];
        const V3 pos{node.pos[0],node.pos[1],node.pos[2]};
        if (node.parent<0) world[i]={pos,basis};
        else {
            const auto &parent=world[node.parent];
            world[i]={parent.pos+parent.basis.Apply(pos),basis*parent.basis};
        }
    }
    return true;
}
