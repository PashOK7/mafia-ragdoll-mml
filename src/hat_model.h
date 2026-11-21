#pragma once
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <utility>
#include <vector>
#include "mathx.h"

namespace hats {
using Bytes = std::vector<uint8_t>;

// Split only rigid, neck-weighted hat face groups. All original vertices, joint
// indices, morph channels and other node indices remain valid in the character.
struct Reader {
    const Bytes &data;
    size_t pos = 0;
    struct Invalid {};
    void skip(size_t n) { if (n > data.size()-pos) throw Invalid{}; pos += n; }
    template<class T> T get() { skip(sizeof(T)); T v; memcpy(&v, data.data()+pos-sizeof(T), sizeof v); return v; }
    std::string str() { auto n=get<uint8_t>(); skip(n); return {reinterpret_cast<const char*>(data.data()+pos-n), n}; }
};
template<class T> void put(Bytes &out, const T &v) {
    const auto *p = reinterpret_cast<const uint8_t*>(&v);
    out.insert(out.end(), p, p+sizeof v);
}
inline void str(Bytes &out, const std::string &v) { put(out,uint8_t(v.size())); out.insert(out.end(),v.begin(),v.end()); }
inline std::string lower(std::string v) {
    for (char &c:v) c=static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return v;
}
inline bool hatTexture(const std::string &name) {
    auto t=lower(name);
    return t.ends_with(".bmp") && (t.find("klb")!=t.npos || t.find("cpc")!=t.npos ||
        t.starts_with("hoolcep") || t.starts_with("tomdcep") || t.starts_with("luccepc"));
}
struct Group { uint16_t material; std::vector<uint16_t> index; };
struct Vertex { V3 pos, normal; float uv[2]; };
struct Lod {
    float distance;
    std::vector<Vertex> vertices;
    std::vector<Group> groups;
    struct Bone { uint32_t first, rigid; std::array<float,16> bind; };
    std::vector<Bone> bones;
};
struct Edit { size_t start,end; uint8_t groups; bool count; };
inline void readMesh(Reader &r, const std::vector<bool> &hat, std::vector<Lod> &lods,
                     std::vector<Edit> &edits, bool skinned) {
    if (r.get<uint16_t>()) return; // instanced visual, mesh stored on its source
    lods.resize(r.get<uint8_t>());
    for (auto &lod:lods) {
        lod.distance=r.get<float>();
        lod.vertices.resize(r.get<uint16_t>());
        for (auto &v:lod.vertices) v=r.get<Vertex>();
        size_t count=r.pos;
        const int groups=r.get<uint8_t>();
        for (int i=0;i<groups;++i) {
            size_t start=r.pos;
            Group group{};
            group.index.resize(size_t(r.get<uint16_t>())*3);
            for (auto &index:group.index) index=r.get<uint16_t>();
            group.material=r.get<uint16_t>();
            if (skinned && group.material<hat.size() && hat[group.material] && !group.index.empty()) {
                edits.push_back({start,r.pos,0,false});
                lod.groups.push_back(std::move(group));
            }
        }
        if (!lod.groups.empty()) edits.push_back({count,count+1,uint8_t(groups-lod.groups.size()),true});
    }
    if (!skinned) return;
    for (auto &lod:lods) {
        lod.bones.resize(r.get<uint8_t>());
        r.skip(28);
        unsigned first=0;
        for (auto &bone:lod.bones) {
            bone.bind=r.get<std::array<float,16>>();
            bone.first=first;
            bone.rigid=r.get<uint32_t>();
            auto blended=r.get<uint32_t>();
            r.skip(28+size_t(blended)*4);
            if (uint64_t(first)+bone.rigid+blended>lod.vertices.size()) throw Reader::Invalid{};
            first+=bone.rigid+blended;
        }
    }
}
inline void skipMorph(Reader &r) {
    int targets=r.get<uint8_t>();
    if (!targets) return;
    int channels=r.get<uint8_t>(), lods=r.get<uint8_t>();
    for (int i=0;i<channels*lods;++i) {
        auto n=r.get<uint16_t>();
        if (!n) continue;
        r.skip(size_t(n)*targets*24);
        if (r.get<uint8_t>()) r.skip(size_t(n)*2);
    }
    r.skip(40);
}
inline Bytes object(const std::vector<Lod> &lods, int joint, V3 &centre) {
    Bytes mesh;
    put(mesh,uint16_t(0));
    put(mesh,uint8_t(lods.size()));
    for (const auto &lod:lods) {
        if (joint<0 || size_t(joint)>=lod.bones.size()) throw Reader::Invalid{};
        const auto &bone=lod.bones[joint];
        M3 basis=M3::Raw(bone.bind.data());
        V3 translation{bone.bind[12],bone.bind[13],bone.bind[14]};
        std::vector<Vertex> vertices;
        auto groups=lod.groups;
        std::vector<int> remap(lod.vertices.size(),-1);
        for (auto &group:groups) for (auto &index:group.index) {
            if (index>=remap.size() || index<bone.first || index-bone.first>=bone.rigid) throw Reader::Invalid{};
            if (remap[index]<0) {
                remap[index]=int(vertices.size());
                auto v=lod.vertices[index];
                v.pos=basis.Apply(v.pos)+translation;
                v.normal=Norm(basis.Apply(v.normal));
                vertices.push_back(v);
            }
            index=uint16_t(remap[index]);
        }
        if (&lod==lods.data()) {
            if (vertices.empty()) throw Reader::Invalid{};
            V3 lo=vertices.front().pos, hi=lo;
            for (const auto &v:vertices) {
                lo={std::min(lo.x,v.pos.x),std::min(lo.y,v.pos.y),std::min(lo.z,v.pos.z)};
                hi={std::max(hi.x,v.pos.x),std::max(hi.y,v.pos.y),std::max(hi.z,v.pos.z)};
            }
            centre=(lo+hi)*.5f;
            if (!(Len(hi-lo)<1.2f && Len(hi-lo)>.08f)) throw Reader::Invalid{};
        }
        put(mesh,lod.distance); put(mesh,uint16_t(vertices.size()));
        for (auto &v:vertices) { v.pos=v.pos-centre; put(mesh,v); }
        put(mesh,uint8_t(groups.size()));
        for (const auto &g:groups) {
            put(mesh,uint16_t(g.index.size()/3));
            for (auto i:g.index) put(mesh,i);
            put(mesh,g.material);
        }
    }
    return mesh;
}
inline void node(Bytes &out, const Bytes &mesh, const std::string &name, uint16_t parent, V3 pos) {
    put(out,uint8_t(1)); put(out,uint8_t(0)); put(out,uint16_t(0x2a00)); put(out,parent);
    put(out,pos); put(out,V3{1,1,1}); put(out,Q{1,0,0,0}); put(out,uint8_t(9));
    str(out,name); str(out,""); out.insert(out.end(),mesh.begin(),mesh.end());
}
inline bool split(const Bytes &source, const std::string &marker, Bytes &person, Bytes &prop) {
    try {
        Reader r{source};
        if (r.get<uint32_t>()!=0x00534434 || r.get<uint16_t>()!=29) return false;
        r.skip(8);
        std::vector<bool> hats(size_t(r.get<uint16_t>())+1);
        for (size_t i=1;i<hats.size();++i) {
            auto f=r.get<uint32_t>(); r.skip(40);
            if (f&(1u<<19)) { r.skip(4); r.str(); }
            bool diffuse=(f&(1u<<18))!=0, opacity=(f&(1u<<30)) && !(f&(1u<<24));
            if (diffuse) hats[i]=hatTexture(r.str());
            if (opacity) r.str();
            if (!diffuse && !opacity) r.skip(1);
            if (f&(3u<<25)) r.skip(18);
        }
        if (std::find(hats.begin(),hats.end(),true)==hats.end()) return false;
        const size_t countAt=r.pos;
        int count=r.get<uint16_t>(), neck=-1, joint=-1;
        std::vector<Edit> edits;
        std::vector<Lod> hatLods;
        if (count<2 || count>=65535) return false;
        for (int i=0;i<count;++i) {
            auto type=r.get<uint8_t>(); int visual=-1;
            if (type==1) { visual=r.get<uint8_t>(); r.skip(2); }
            r.skip(43); auto name=lower(r.str()); r.str();
            if (name.starts_with("rdhat_")) return false;
            if (type==1 && ((visual>=0 && visual<=3) || visual==5)) {
                std::vector<Lod> lods;
                readMesh(r,hats,lods,edits,visual==2 || visual==3);
                if (std::any_of(lods.begin(),lods.end(),[](auto &l){return !l.groups.empty();})) {
                    if (!hatLods.empty()) return false;
                    const auto &first=lods.front().groups;
                    if (first.empty()) return false;
                    // Lower detail people can bind the same hat to the chest.
                    // Replace those same materials with the neck-rigid original.
                    for (auto &lod:lods) for (auto &g:lod.groups)
                        if (std::none_of(first.begin(),first.end(),[&](auto &f){return f.material==g.material;})) return false;
                    lods.resize(1);
                    lods.front().distance=0;
                    hatLods=std::move(lods);
                }
                if (visual==3 || visual==5) skipMorph(r);
            } else if (type==10) {
                r.skip(64); auto id=r.get<uint32_t>();
                if (name=="neck") { neck=i; joint=int(id); }
            } else if (type==6) r.skip(24);
            else if (type==7) { r.skip(2); r.skip(size_t(r.get<uint8_t>())*2); }
            else return false;
        }
        if (neck<0 || hatLods.empty()) return false;
        V3 centre{};
        Bytes mesh=object(hatLods,joint,centre);
        prop.assign(source.begin(),source.begin()+countAt);
        person=prop; put(prop,uint16_t(1)); put(person,uint16_t(count+1));
        node(prop,mesh,"hat",0,{});
        put(prop,uint8_t(0)); // no model animation
        std::sort(edits.begin(),edits.end(),[](auto &a,auto &b){return a.start<b.start;});
        size_t cursor=countAt+2;
        for (const auto &edit:edits) {
            person.insert(person.end(),source.begin()+cursor,source.begin()+edit.start);
            if (edit.count) put(person,edit.groups);
            cursor=edit.end;
        }
        person.insert(person.end(),source.begin()+cursor,source.begin()+r.pos);
        node(person,mesh,marker,uint16_t(neck+1),centre);
        person.insert(person.end(),source.begin()+r.pos,source.end());
        return true;
    } catch (Reader::Invalid &) { return false; }
}
} // namespace hats
