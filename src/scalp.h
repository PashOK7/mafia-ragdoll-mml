// Mesh edits for the head underneath a detachable hat. Included after the
// 4DS reader's Lod and Edit definitions.
#pragma once
namespace hats {
namespace scalp {
using Point = std::array<float,3>;
struct Edge { uint16_t a,b; };
using Loop = std::vector<Edge>;

inline Point key(V3 p) {
    return {std::round(p.x*100000),std::round(p.y*100000),std::round(p.z*100000)};
}
struct Bounds {
    V3 lo{1e6f,1e6f,1e6f}, hi{-1e6f,-1e6f,-1e6f};
    void add(V3 p) {
        lo={std::min(lo.x,p.x),std::min(lo.y,p.y),std::min(lo.z,p.z)};
        hi={std::max(hi.x,p.x),std::max(hi.y,p.y),std::max(hi.z,p.z)};
    }
    V3 middle() const {return (lo+hi)*.5f;}
};

// UV seams have duplicate vertex indices. Match edges in bind space while
// retaining their original corner indices for skinning, UVs and winding.
inline std::vector<Loop> boundaries(const Lod &lod,const Group &face) {
    const auto point=[&](uint16_t i){return key(lod.vertices[i].pos);};
    struct Use { unsigned count{}; Edge edge{}; };
    std::map<std::pair<Point,Point>,Use> uses;
    for (size_t i=0;i<face.index.size();i+=3) {
        const Point a=point(face.index[i]), b=point(face.index[i+1]), c=point(face.index[i+2]);
        if (a==b || a==c || b==c) continue;
        for (int j=0;j<3;++j) {
            Edge e{face.index[i+j],face.index[i+(j+1)%3]};
            auto &use=uses[std::minmax(point(e.a),point(e.b))];
            ++use.count; use.edge=e;
        }
    }
    std::map<Point,std::map<Point,Edge>> open;
    auto insert=[&](Edge e){
        Point a=point(e.a),b=point(e.b);
        open[a][b]=e; open[b][a]={e.b,e.a};
    };
    for (auto &[_,u]:uses) if (u.count==1) insert(u.edge);
    std::vector<Loop> loops;
    while (!open.empty()) {
        Loop path;
        std::map<Point,size_t> visited;
        Point p=open.begin()->first;
        for (;;) {
            auto [it,added]=visited.emplace(p,path.size());
            if (!added) {
                Loop cycle(path.begin()+it->second,path.end());
                float area=0;
                for (auto e:cycle) {
                    auto a=lod.vertices[e.a].pos,b=lod.vertices[e.b].pos;
                    area+=a.x*b.z-a.z*b.x;
                }
                if (area<0) {
                    std::reverse(cycle.begin(),cycle.end());
                    for (auto &e:cycle) std::swap(e.a,e.b);
                }
                loops.push_back(std::move(cycle));
                for (size_t i=0;i<it->second;++i) insert(path[i]);
                break;
            }
            auto next=open.find(p);
            if (next==open.end()) break;
            auto edge=next->second.begin()->second;
            Point q=point(edge.b);
            path.push_back(edge);
            next->second.erase(q);
            if (next->second.empty()) open.erase(next);
            auto back=open.find(q);
            back->second.erase(p);
            if (back->second.empty()) open.erase(back);
            p=q;
        }
    }
    return loops;
}
inline bool surrounds(const Lod &lod,const Loop &loop,V3 p) {
    bool inside=false;
    for (auto e:loop) {
        V3 a=lod.vertices[e.a].pos,b=lod.vertices[e.b].pos;
        if ((a.z>p.z)!=(b.z>p.z) && p.x<a.x+(p.z-a.z)*(b.x-a.x)/(b.z-a.z)) inside=!inside;
    }
    return inside;
}

inline float hatCeiling(const Lod &hat,V3 p,float top) {
    // Upper surface of the source hat above this point, including indentations.
    float ceiling=-1e6f;
    for (const auto &g:hat.groups) for (size_t i=0;i<g.index.size();i+=3) {
        const V3 a=hat.vertices[g.index[i]].pos;
        const V3 u=hat.vertices[g.index[i+1]].pos-a,v=hat.vertices[g.index[i+2]].pos-a;
        const float det=u.x*v.z-v.x*u.z;
        if (std::abs(det)<1e-9f) continue;
        const float s=((p.x-a.x)*v.z-(p.z-a.z)*v.x)/det;
        const float t=(u.x*(p.z-a.z)-u.z*(p.x-a.x))/det;
        if (s>=0 && t>=0 && s+t<=1) ceiling=std::max(ceiling,a.y+s*u.y+t*v.y);
    }
    return ceiling>-1e5f ? std::min(top,ceiling-.004f) : top;
}

inline std::vector<uint16_t> available(const Lod &lod,const Loop &rim) {
    // Keep every slot referenced by a retained triangle or facial morph.
    // Matching the rim's rigid bone also handles the simpler distant skeleton.
    std::set<uint16_t> slots;
    for (auto &g:lod.groups) slots.insert(g.index.begin(),g.index.end());
    for (auto &g:lod.retained) for (auto i:g.index) slots.erase(i);
    for (auto i:lod.morphVertices) slots.erase(i);
    int best=0;
    const Lod::Bone *bone=nullptr;
    for (auto &b:lod.bones) {
        int votes=0;
        for (auto e:rim) votes+=e.a>=b.first && e.a-b.first<b.rigid;
        if (votes>best) {best=votes;bone=&b;}
    }
    std::vector<uint16_t> result;
    if (bone) for (auto i:slots) if (i>=bone->first && i-bone->first<bone->rigid) result.push_back(i);
    return result;
}

inline void closeRim(const Lod &lod,const Lod &hat,const Group &face,const Loop &rim,
                     const std::vector<Loop> &loops,std::vector<Edit> &edits) {
    Bounds bounds;
    std::set<Point> seam;
    std::set<uint16_t> corners;
    for (auto e:rim) {
        bounds.add(lod.vertices[e.a].pos);
        seam.insert(key(lod.vertices[e.a].pos));
        corners.insert(e.a); corners.insert(e.b);
    }
    V3 centre=bounds.middle();
    const V3 radius=(bounds.hi-bounds.lo)*.5f;
    const float top=hatCeiling(hat,centre,centre.y+std::sqrt(radius.x*radius.z));
    auto slots=available(lod,rim);
    std::map<uint16_t,Vertex> added;
    std::vector<uint16_t> indices;
    auto triangle=[&](uint16_t a,uint16_t b,uint16_t c){indices.insert(indices.end(),{a,b,c});};
    auto write=[&](const Vertex &v){
        auto i=slots.back(); slots.pop_back();
        added[i]=v;
        return i;
    };
    auto disk=[&](const Loop &loop){
        for (size_t i=1;i+1<loop.size();++i) triangle(loop[i].b,loop[i].a,loop.front().a);
    };
    if (slots.empty()) {
        // A few distant LODs leave a small hole even without any hat vertices.
        disk(rim);
    } else {
        const unsigned rings=std::min<size_t>(2,(slots.size()-1)/corners.size());
        Loop previous=rim;
        for (unsigned layer=1;layer<=rings;++layer) {
            const float angle=1.57079633f*layer/(rings+1);
            float rise=std::sin(angle), width=std::cos(angle);
            std::map<uint16_t,uint16_t> inner;
            for (auto i:corners) {
                Vertex v=lod.vertices[i];
                const V3 offset=v.pos-centre;
                v.pos.x=centre.x+offset.x*width; v.pos.z=centre.z+offset.z*width;
                v.pos.y=hatCeiling(hat,v.pos,v.pos.y+(top-v.pos.y)*rise);
                v.normal=Norm({offset.x/radius.x*width,rise,offset.z/radius.z*width});
                v.uv[1]=v.uv[1]+(.035f-v.uv[1])*rise;
                inner[i]=write(v);
            }
            for (size_t j=0;j<rim.size();++j) {
                Edge a=previous[j],b{inner[rim[j].a],inner[rim[j].b]};
                triangle(a.b,a.a,b.a); triangle(a.b,b.a,b.b); previous[j]=b;
            }
        }
        bool splitUV=slots.size()>=rim.size();
        uint16_t pole=0;
        for (size_t j=0;j<rim.size();++j) {
            if (!j || splitUV) {
                float u=(lod.vertices[rim[j].a].uv[0]+lod.vertices[rim[j].b].uv[0])*.5f;
                pole=write({{centre.x,top,centre.z},{0,1,0},{u,.035f}});
            }
            triangle(previous[j].b,previous[j].a,pole);
        }
    }
    // Some meshes have a second tiny cycle touching the hairline at the ear.
    for (const auto &loop:loops) if (&loop!=&rim && loop.size()>=3 &&
        std::any_of(loop.begin(),loop.end(),[&](Edge e){return seam.contains(key(lod.vertices[e.a].pos));}))
        disk(loop);
    // A straight scalp triangle can cross a crease in a fedora even if its
    // three corners fit. Fit interior points too. Original rim vertices are
    // immutable and coincident UV copies receive the same displacement.
    std::map<Point,float> lowering;
    for (size_t i=0;i<indices.size();i+=3) {
        V3 p[3]; bool movable[3];
        for (int k=0;k<3;++k) {
            auto id=indices[i+k];
            movable[k]=added.contains(id);
            p[k]=movable[k] ? added.at(id).pos : lod.vertices[id].pos;
        }
        for (int a=1;a<=4;++a) for (int b=1;b<6-a;++b) {
            const float w[3]={a/6.f,b/6.f,(6-a-b)/6.f};
            const V3 sample=p[0]*w[0]+p[1]*w[1]+p[2]*w[2];
            const float excess=sample.y-hatCeiling(hat,sample,sample.y);
            const float share=w[0]*movable[0]+w[1]*movable[1]+w[2]*movable[2];
            if (excess<=0 || share==0) continue;
            for (int k=0;k<3;++k) if (movable[k]) {
                float &shift=lowering[key(p[k])];
                // A distant LOD's original forehead can already be outside
                // the full-detail brim. Never pull the new surface below it.
                shift=std::max(shift,std::min(excess/share,std::max(0.f,p[k].y-bounds.lo.y)));
            }
        }
    }
    for (auto &[id,v]:added) {
        v.pos.y-=lowering[key(v.pos)];
        size_t at=lod.vertexStart+size_t(id)*sizeof(Vertex);
        Edit edit{at,at+sizeof(Vertex),{}}; put(edit.replacement,v);
        edits.push_back(std::move(edit));
    }
    if ((indices.size()+face.index.size())/3>65535) throw Reader::Invalid{};
    Edit edit{face.countAt,face.countAt+2,{}};
    put(edit.replacement,uint16_t((indices.size()+face.index.size())/3));
    for (auto i:indices) put(edit.replacement,i);
    edits.push_back(std::move(edit));
}
} // namespace scalp
inline void removeHatFaces(std::vector<Lod> &lods,std::vector<Edit> &edits) {
    scalp::Bounds bounds;
    for (auto &g:lods.front().groups) for (auto i:g.index) bounds.add(lods.front().vertices[i].pos);
    // Lower LODs can assign the hat texture to both hands. Keep faces
    // outside the original headwear's neighbourhood on the animated skin.
    const V3 pad=(bounds.hi-bounds.lo)*.5f;
    const V3 lo=bounds.lo-pad, hi=bounds.hi+pad;
    for (auto &lod:lods) {
        unsigned count=lod.retained.size()+lod.groups.size();
        for (auto &g:lod.groups) {
            Group other{g.material,{},g.countAt};
            std::vector<uint16_t> head;
            for (size_t j=0;j<g.index.size();j+=3) {
                auto first=g.index.begin()+j;
                bool atHead=std::all_of(first,first+3,[&](uint16_t i){
                    auto p=lod.vertices[i].pos;
                    return p.x>=lo.x && p.x<=hi.x && p.y>=lo.y && p.y<=hi.y && p.z>=lo.z && p.z<=hi.z;
                });
                auto &dst=atHead ? head : other.index;
                dst.insert(dst.end(),first,first+3);
            }
            if (!head.empty()) {
                Edit e{g.countAt,g.countAt+4+g.index.size()*2,{}};
                if (other.index.empty()) --count;
                else {
                    put(e.replacement,uint16_t(other.index.size()/3));
                    for (auto i:other.index) put(e.replacement,i);
                    put(e.replacement,other.material);
                }
                edits.push_back(std::move(e));
            }
            if (!other.index.empty()) lod.retained.push_back(std::move(other));
            g.index=std::move(head);
        }
        std::erase_if(lod.groups,[](const Group &g){return g.index.empty();});
        edits.push_back({lod.groupCountAt,lod.groupCountAt+1,{uint8_t(count)}});
    }
}
inline void repairCrowns(const std::vector<Lod> &lods,std::vector<Edit> &edits) {
    using namespace scalp;
    Bounds hat;
    for (const auto &g:lods.front().groups) for (auto i:g.index) hat.add(lods.front().vertices[i].pos);
    const V3 axis=hat.middle();
    uint16_t faceMaterial=0;
    for (const auto &lod:lods) {
        for (const auto &face:lod.retained) {
            if (faceMaterial && face.material!=faceMaterial) continue;
            const auto loops=boundaries(lod,face);
            const Loop *best=nullptr;
            float height=std::max(hat.lo.y-.045f,hat.hi.y-.18f);
            for (const auto &loop:loops) {
                if (loop.size()<3 || !surrounds(lod,loop,axis)) continue;
                float y=0;
                for (auto e:loop) y+=lod.vertices[e.a].pos.y;
                y/=loop.size();
                if (y>height) {height=y;best=&loop;}
            }
            if (best) {
                closeRim(lod,lod.groups.empty() ? lods.front() : lod,face,*best,loops,edits);
                faceMaterial=face.material;
                break;
            }
        }
    }
}
} // namespace hats
