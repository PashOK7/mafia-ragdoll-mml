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

// Separate the hat's motion from the neck while keeping its faces in the native
// character skin. Original vertex indices, joint indices and morphs stay valid.
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
    size_t vertexStart{}, groupCountAt{}, groupsEnd{}, skinStart{}, skinEnd{};
    uint32_t unweighted{};
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
        lod.groupsEnd=r.pos;
    }
    if (!skinned) return;
    for (auto &lod:lods) {
        lod.skinStart=r.pos;
        lod.bones.resize(r.get<uint8_t>());
        lod.unweighted=r.get<uint32_t>();
        r.skip(24);
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
        if (uint64_t(first)+lod.unweighted!=lod.vertices.size()) throw Reader::Invalid{};
        lod.skinEnd=r.pos;
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
// A small shared cloth texture with a leather sweatband along the opening.
// Windows BMP rows start at the bottom. The lining UV is measured from the brim.
inline Bytes liningBitmap() {
    constexpr uint32_t side=64, pixels=side*side*3;
    Bytes bmp;
    put(bmp,uint16_t(0x4d42)); put(bmp,uint32_t(54+pixels)); put(bmp,uint32_t(0)); put(bmp,uint32_t(54));
    put(bmp,uint32_t(40)); put(bmp,side); put(bmp,side); put(bmp,uint16_t(1)); put(bmp,uint16_t(24));
    put(bmp,uint32_t(0)); put(bmp,pixels);
    for (int i=0;i<4;++i) put(bmp,uint32_t(0));
    for (unsigned y=0;y<side;++y) for (unsigned x=0;x<side;++x) {
        const bool band=y<14;
        const int stitch=(y==11 && (x%8)<5) ? 12 : 0;
        const int weave=((x+y)%4==0 ? 2 : -1);
        const int shade=band ? 0 : int(24*(side-1-y)/(side-14));
        const int tone=shade+stitch+weave;
        bmp.push_back(uint8_t((band ? 29 : 45)+tone));
        bmp.push_back(uint8_t((band ? 35 : 50)+tone));
        bmp.push_back(uint8_t((band ? 44 : 57)+tone));
    }
    return bmp;
}
// Keep the opening at the brim. An inward-facing shell follows even a folded
// crown or a cap's peak. Stitch only real boundaries, ignoring UV seams.
inline void lineProp(std::vector<Vertex> &vertices, std::vector<Group> &groups, V3 centre, uint16_t material) {
    size_t count=vertices.size();
    if (count>32767 || groups.size()>=255) throw Reader::Invalid{};
    Group lining{material,{}};
    using Edge=std::pair<scalp::Point,scalp::Point>;
    struct Use { unsigned count{}; uint16_t a{},b{}; };
    std::map<Edge,Use> edges;
    scalp::Bounds bounds;
    for (const Vertex &v:vertices) bounds.add(v.pos);
    for (size_t i=0;i<count;++i) {
        auto v=vertices[i];
        float height=(v.pos.y-bounds.lo.y)/std::max(.001f,bounds.hi.y-bounds.lo.y);
        v.uv[0]=.5f+std::atan2(v.pos.z-centre.z,v.pos.x-centre.x)/6.2831853f;
        v.uv[1]=.01f+.98f*height;
        v.pos=centre+(v.pos-centre)*.985f;
        v.normal=-v.normal;
        vertices.push_back(v);
    }
    auto inner=[count](uint16_t i){return uint16_t(count+i);};
    for (const auto &group:groups) for (size_t i=0;i<group.index.size();i+=3) {
        const auto *t=group.index.data()+i;
        lining.index.insert(lining.index.end(),{inner(t[2]),inner(t[1]),inner(t[0])});
        for (int j=0;j<3;++j) {
            auto a=t[j],b=t[(j+1)%3];
            auto pa=scalp::key(vertices[a].pos),pb=scalp::key(vertices[b].pos);
            if (pa==pb) continue;
            auto &edge=edges[std::minmax(pa,pb)];
            ++edge.count; edge.a=a; edge.b=b;
        }
    }
    for (auto &[_,edge]:edges) if (edge.count==1) {
        auto a=edge.a,b=edge.b;
        lining.index.insert(lining.index.end(),{b,a,inner(a),b,inner(a),inner(b)});
    }
    if (lining.index.size()/3>65535) throw Reader::Invalid{};
    groups.push_back(std::move(lining));
}
struct PropMesh {
    std::vector<V3> vertices;
    std::vector<uint16_t> indices;
    float radius=0;
};
struct HatGeometry {
    std::vector<Vertex> vertices;
    std::vector<Group> groups;
};
inline HatGeometry neckGeometry(const Lod &lod, const Lod::Bone &neck, bool fullDetail) {
    HatGeometry geometry{{},lod.groups};
    const M3 basis=M3::Raw(neck.bind.data());
    const V3 translation{neck.bind[12],neck.bind[13],neck.bind[14]};
    std::vector<int> remap(lod.vertices.size(),-1);
    for (auto &group:geometry.groups) for (auto &index:group.index) {
        if (index>=remap.size() || (fullDetail && (index<neck.first || index-neck.first>=neck.rigid)))
            throw Reader::Invalid{};
        if (remap[index]<0) {
            remap[index]=int(geometry.vertices.size());
            auto v=lod.vertices[index];
            v.pos=basis.Apply(v.pos)+translation;
            v.normal=Norm(basis.Apply(v.normal));
            geometry.vertices.push_back(v);
        }
        index=uint16_t(remap[index]);
    }
    return geometry;
}
inline Bytes looseObject(HatGeometry geometry, V3 centre, uint16_t lining, PropMesh &collision) {
    auto &[vertices,groups]=geometry;
    lineProp(vertices,groups,centre,lining);
    Bytes mesh;
    put(mesh,uint16_t(0));
    put(mesh,uint8_t(1)); put(mesh,0.f); put(mesh,uint16_t(vertices.size()));
    for (auto &v:vertices) {
        v.pos=v.pos-centre;
        put(mesh,v);
        collision.vertices.push_back(v.pos);
        collision.radius=std::max(collision.radius,Len(v.pos));
    }
    put(mesh,uint8_t(groups.size()));
    for (const auto &g:groups) {
        put(mesh,uint16_t(g.index.size()/3));
        for (auto i:g.index) put(mesh,i);
        put(mesh,g.material);
        collision.indices.insert(collision.indices.end(),g.index.begin(),g.index.end());
    }
    return mesh;
}

constexpr std::array<float,16> identity{1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
inline void jointSkin(Bytes &out,uint32_t count,const scalp::Bounds &bounds) {
    put(out,identity);
    put(out,count); put(out,uint32_t(0)); put(out,uint32_t(0));
    put(out,bounds.lo); put(out,bounds.hi);
}
inline void mountInSkin(const Lod &lod,HatGeometry geometry,V3 centre,std::vector<Edit> &edits) {
    const auto &[vertices,groups]=geometry;
    const size_t first=lod.vertices.size();
    if (first+vertices.size()>65535 || lod.retained.size()+groups.size()>255) throw Reader::Invalid{};
    Edit count{lod.vertexStart-2,lod.vertexStart,{}};
    put(count.replacement,uint16_t(first+vertices.size()));
    edits.push_back(std::move(count));
    Edit geometryEdit{lod.groupCountAt,lod.groupCountAt+1,{}};
    scalp::Bounds hatBounds;
    for (auto v:vertices) {
        v.pos=v.pos-centre;
        put(geometryEdit.replacement,v);
        hatBounds.add(v.pos);
    }
    put(geometryEdit.replacement,uint8_t(lod.retained.size()+groups.size()));
    edits.push_back(std::move(geometryEdit));
    Edit faces{lod.groupsEnd,lod.groupsEnd,{}};
    for (const Group &group:groups) {
        put(faces.replacement,uint16_t(group.index.size()/3));
        for (uint16_t index:group.index) put(faces.replacement,uint16_t(first+index));
        put(faces.replacement,group.material);
    }
    edits.push_back(std::move(faces));

    // A joint at mesh identity consumes the original rigid tail in place.
    // That keeps face and indexed or implicit morph references at their stock indices.
    Edit skin{lod.skinStart,lod.skinStart+5,{}};
    put(skin.replacement,uint8_t(lod.bones.size()+2));
    put(skin.replacement,uint32_t(0));
    edits.push_back(std::move(skin));
    scalp::Bounds fixedBounds;
    for (size_t i=first-lod.unweighted;i<first;++i) fixedBounds.add(lod.vertices[i].pos);
    Edit weights{lod.skinEnd,lod.skinEnd,{}};
    jointSkin(weights.replacement,lod.unweighted,fixedBounds);
    jointSkin(weights.replacement,uint32_t(vertices.size()),hatBounds);
    edits.push_back(std::move(weights));
}

inline void node(Bytes &out, const Bytes &mesh, const std::string &name, uint16_t parent, V3 pos) {
    put(out,uint8_t(1)); put(out,uint8_t(0)); put(out,uint16_t(0x2a00)); put(out,parent);
    put(out,pos); put(out,V3{1,1,1}); put(out,Q{1,0,0,0}); put(out,uint8_t(9));
    str(out,name); str(out,""); out.insert(out.end(),mesh.begin(),mesh.end());
}
inline void jointNode(Bytes &out,const std::string &name,uint16_t parent,V3 pos,uint32_t index) {
    put(out,uint8_t(10)); put(out,parent);
    put(out,pos); put(out,V3{1,1,1}); put(out,Q{1,0,0,0}); put(out,uint8_t(9));
    str(out,name); str(out,""); put(out,identity); put(out,index);
}
inline bool split(const Bytes &source, const std::string &marker, Bytes &person, Bytes &prop,V3 &dimensions,PropMesh &collision) {
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
        int count=r.get<uint16_t>(), neck=-1, joint=-1, skin=-1;
        std::vector<Edit> edits;
        std::vector<Lod> hatLods;
        std::vector<uint16_t> parents;
        std::vector<std::pair<int,uint32_t>> joints;
        if (count<2 || count>65533) return false;
        for (int i=0;i<count;++i) {
            auto type=r.get<uint8_t>(); int visual=-1;
            if (type==1) { visual=r.get<uint8_t>(); r.skip(2); }
            parents.push_back(r.get<uint16_t>());
            r.skip(41); auto name=lower(r.str()); r.str();
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
                    skin=i+1;
                    hatLods=std::move(lods);
                }
            } else if (type==10) {
                r.skip(64); auto id=r.get<uint32_t>();
                joints.emplace_back(i+1,id);
                if (name=="neck") { neck=i+1; joint=int(id); }
            } else if (type==6) r.skip(24);
            else if (type==7) { r.skip(2); r.skip(size_t(r.get<uint8_t>())*2); }
            else return false;
        }
        if (neck<0 || hatLods.empty()) return false;
        const auto nativeJoints=hatLods.front().bones.size();
        if (joint<0 || size_t(joint)>=nativeJoints || nativeJoints>62) return false;
        std::vector<bool> seen(nativeJoints);
        for (auto [frame,id]:joints) {
            int parent=frame;
            for (int depth=0; parent>0 && parent<=count && parent!=skin && depth<count; ++depth)
                parent=parents[parent-1];
            if (parent!=skin) {
                if (frame==neck) return false;
                continue;
            }
            if (id>=nativeJoints || seen[id]) return false;
            seen[id]=true;
        }
        if (std::find(seen.begin(),seen.end(),false)!=seen.end()) return false;
        if (std::any_of(hatLods.begin(),hatLods.end(),[&](const Lod &lod){return lod.bones.size()!=nativeJoints;}))
            return false;
        removeHatFaces(hatLods,edits);
        scalp::Bounds shell;
        for (const auto &g:hatLods.front().groups) for (auto i:g.index) shell.add(hatLods.front().vertices[i].pos);
        dimensions=shell.hi-shell.lo;
        repairCrowns(hatLods,edits);
        const auto &neckBind=hatLods.front().bones[joint];
        auto full=neckGeometry(hatLods.front(),neckBind,true);
        if (full.vertices.empty()) return false;
        scalp::Bounds local;
        for (const auto &v:full.vertices) local.add(v.pos);
        const auto extent=Len(local.hi-local.lo);
        if (!(extent>.08f && extent<1.2f)) return false;
        const V3 centre=local.middle();
        if (hats.size()>65535) return false;
        const auto lining=uint16_t(hats.size());
        Bytes mesh=looseObject(full,centre,lining,collision);
        for (const auto &lod:hatLods)
            mountInSkin(lod,neckGeometry(lod,neckBind,false),centre,edits);
        person.assign(source.begin(),source.begin()+countAt);
        prop=person;
        memcpy(prop.data()+14,&lining,sizeof lining); // 4DS material count
        put(prop,uint32_t(1u<<18));
        put(prop,V3{.65f,.65f,.65f}); put(prop,V3{.85f,.85f,.85f});
        put(prop,V3{}); put(prop,1.f); str(prop,"rd_lining.bmp");
        put(prop,uint16_t(1)); put(person,uint16_t(count+2));
        node(prop,mesh,"hat",0,{});
        put(prop,uint8_t(0)); // no model animation
        std::stable_sort(edits.begin(),edits.end(),[](const Edit &a,const Edit &b){
            return a.start==b.start ? a.end<b.end : a.start<b.start;
        });
        size_t cursor=countAt+2;
        for (const auto &edit:edits) {
            if (edit.start<cursor || edit.end<edit.start || edit.end>r.pos) return false;
            person.insert(person.end(),source.begin()+cursor,source.begin()+edit.start);
            person.insert(person.end(),edit.replacement.begin(),edit.replacement.end());
            cursor=edit.end;
        }
        person.insert(person.end(),source.begin()+cursor,source.begin()+r.pos);
        jointNode(person,"rd_rigid_tail",uint16_t(skin),{},uint32_t(nativeJoints));
        jointNode(person,marker,uint16_t(neck),centre,uint32_t(nativeJoints+1));
        person.insert(person.end(),source.begin()+r.pos,source.end());
        return true;
    } catch (Reader::Invalid &) { return false; }
}
} // namespace hats
