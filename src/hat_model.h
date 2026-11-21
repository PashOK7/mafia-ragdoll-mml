#pragma once
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <set>
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
struct Group { uint16_t material; std::vector<uint16_t> index; size_t countAt{}; };
struct Vertex { V3 pos, normal; float uv[2]; };
struct Lod {
    float distance;
    std::vector<Vertex> vertices;
    std::vector<Group> groups;
    std::vector<Group> retained;
    std::set<uint16_t> morphVertices;
    size_t vertexStart{}, groupCountAt{};
    struct Bone { uint32_t first, rigid; std::array<float,16> bind; };
    std::vector<Bone> bones;
};
struct Edit { size_t start,end; Bytes replacement; };
inline void readMesh(Reader &r, const std::vector<bool> &hat, std::vector<Lod> &lods, bool skinned) {
    if (r.get<uint16_t>()) return; // instanced visual, mesh stored on its source
    lods.resize(r.get<uint8_t>());
    for (auto &lod:lods) {
        lod.distance=r.get<float>();
        lod.vertices.resize(r.get<uint16_t>());
        lod.vertexStart=r.pos;
        for (auto &v:lod.vertices) v=r.get<Vertex>();
        lod.groupCountAt=r.pos;
        const int groups=r.get<uint8_t>();
        for (int i=0;i<groups;++i) {
            size_t start=r.pos;
            Group group{};
            group.countAt=start;
            group.index.resize(size_t(r.get<uint16_t>())*3);
            for (auto &index:group.index) {
                index=r.get<uint16_t>();
                if (index>=lod.vertices.size()) throw Reader::Invalid{};
            }
            group.material=r.get<uint16_t>();
            if (skinned && group.material<hat.size() && hat[group.material] && !group.index.empty()) {
                lod.groups.push_back(std::move(group));
            } else if (skinned) lod.retained.push_back(std::move(group));
        }
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
inline void readMorph(Reader &r, std::vector<Lod> &meshes) {
    int targets=r.get<uint8_t>();
    if (!targets) return;
    int channels=r.get<uint8_t>(), lods=r.get<uint8_t>();
    for (int l=0;l<lods;++l) for (int c=0;c<channels;++c) {
        auto n=r.get<uint16_t>();
        if (!n) continue;
        r.skip(size_t(n)*targets*24);
        bool indexed=r.get<uint8_t>()!=0;
        for (unsigned j=0;j<n;++j) {
            auto index=indexed ? r.get<uint16_t>() : uint16_t(j);
            if (l<int(meshes.size())) meshes[l].morphVertices.insert(index);
        }
    }
    r.skip(40);
}
} // namespace hats
#include "scalp.h"
namespace hats {
inline Bytes object(const std::vector<Lod> &lods, int joint, V3 &centre, bool worn) {
    Bytes mesh;
    put(mesh,uint16_t(0));
    const size_t count=worn ? lods.size() : 1;
    put(mesh,uint8_t(count));
    if (joint<0 || size_t(joint)>=lods.front().bones.size()) throw Reader::Invalid{};
    const auto &bone=lods.front().bones[joint];
    for (size_t level=0;level<count;++level) {
        const auto &lod=lods[level];
        M3 basis=M3::Raw(bone.bind.data());
        V3 translation{bone.bind[12],bone.bind[13],bone.bind[14]};
        std::vector<Vertex> vertices;
        auto groups=lod.groups;
        std::vector<int> remap(lod.vertices.size(),-1);
        for (auto &group:groups) for (auto &index:group.index) {
            if (index>=remap.size() || (!level && (index<bone.first || index-bone.first>=bone.rigid))) throw Reader::Invalid{};
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
        if (vertices.empty()) {
            // Native objects keep a valid draw range even at a distance where
            // the original skin omits the hat. This LOD has zero surface area.
            vertices.push_back({centre,{0,1,0},{0,0}});
            groups.push_back({lods.front().groups.front().material,{0,0,0}});
        }
        put(mesh,worn ? lod.distance : 0.f); put(mesh,uint16_t(vertices.size()));
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
                readMesh(r,hats,lods,visual==2 || visual==3);
                if (visual==3 || visual==5) readMorph(r,lods);
                if (std::any_of(lods.begin(),lods.end(),[](auto &l){return !l.groups.empty();})) {
                    if (!hatLods.empty()) return false;
                    const auto &first=lods.front().groups;
                    if (first.empty()) return false;
                    // Lower detail people can bind the same hat to the chest.
                    // Replace those same materials with the neck-rigid original.
                    for (auto &lod:lods) for (auto &g:lod.groups)
                        if (std::none_of(first.begin(),first.end(),[&](auto &f){return f.material==g.material;})) return false;
                    hatLods=std::move(lods);
                }
            } else if (type==10) {
                r.skip(64); auto id=r.get<uint32_t>();
                if (name=="neck") { neck=i; joint=int(id); }
            } else if (type==6) r.skip(24);
            else if (type==7) { r.skip(2); r.skip(size_t(r.get<uint8_t>())*2); }
            else return false;
        }
        if (neck<0 || hatLods.empty()) return false;
        removeHatFaces(hatLods,edits);
        repairCrowns(hatLods,edits);
        V3 centre{};
        Bytes mesh=object(hatLods,joint,centre,false);
        prop.assign(source.begin(),source.begin()+countAt);
        person=prop; put(prop,uint16_t(1)); put(person,uint16_t(count+1));
        node(prop,mesh,"hat",0,{});
        put(prop,uint8_t(0)); // no model animation
        std::sort(edits.begin(),edits.end(),[](auto &a,auto &b){return a.start<b.start;});
        size_t cursor=countAt+2;
        for (const auto &edit:edits) {
            if (edit.start<cursor || edit.end<edit.start || edit.end>r.pos) return false;
            person.insert(person.end(),source.begin()+cursor,source.begin()+edit.start);
            person.insert(person.end(),edit.replacement.begin(),edit.replacement.end());
            cursor=edit.end;
        }
        person.insert(person.end(),source.begin()+cursor,source.begin()+r.pos);
        auto worn=object(hatLods,joint,centre,true);
        node(person,worn,marker,uint16_t(neck+1),centre);
        person.insert(person.end(),source.begin()+r.pos,source.end());
        return true;
    } catch (Reader::Invalid &) { return false; }
}
} // namespace hats
