#include <algorithm>
#include <vector>
#include <list>
#include <iostream>
#include <set>
#include <map>

class AllError:public std::exception
{
	std::string str;
public:
	AllError(const char*p):str(p) {
	}

	~AllError() throw() {
	}

	const char* what()const throw() {
		return str.c_str();
	}
};

typedef int TokenMovePosition;
typedef std::vector<TokenMovePosition> TokenFree;

class TokenPosition16:public std::vector<int> {
public:
	TokenPosition16() {
		reserve(16);
	}

	const_iterator Find(int k) const {
		const_iterator i=begin();
		for(;i!=end();i++) {
			if(*i==k)
				return i;
		}
		return i;
	};

	void Reset() {
		assign(16,0);
		for(int i=0;i<size();i++) {
			at(i)=i+1;
		}
	}

	int Calc(const TokenPosition16 &goal) const {
	
		//int err=0;
		//for(int i=0;i<16;i++)
		//{
		// err+=at(i)!=goal.at(i);
		//}

		// manhattan distance
		int err=0;
		for(int i=0;i<16;i++) {
		
			int goalpos=std::distance(goal.begin(),goal.Find(at(i)));
			int cols=std::abs(goalpos%4-i%4);
			int rows=std::abs(goalpos/4-i/4);
			err+=cols+rows;
		}
		return err;
	}

	bool Goal(const TokenPosition16&t) const {
	
	const int icount=16;
		if(t.size()!=icount || size()!=icount)
		throw AllError("Wrong tokens");
		int i=0;
		int ret=0;
		while(i<icount && !(ret=at(i)-t.at(i))) {
			i++;
		};
		return ret==0;
	}

	void Random() {
		clear();
		int tokens[]={2,11,14,5, 9,7,16,4, 13,12,1,3, 8,10,15,6};assign(&tokens[0],&tokens[sizeof(tokens)/sizeof(int)]);
		//int tokens[]={1,2,3,4, 5,6,7,8, 9,10,11,12, 13,14,15,16};assign(&tokens[0],&tokens[sizeof(tokens)/sizeof(int)]);
		//int tokens[]={1,2,3,4, 5,6,7,8, 9,10,11,16, 13,14,15,12};assign(&tokens[0],&tokens[sizeof(tokens)/sizeof(int)]);
		//int tokens[]={1,2,3,4, 5,6,7,8, 9,10,16,11, 13,14,15,12};assign(&tokens[0],&tokens[sizeof(tokens)/sizeof(int)]);
	};

	TokenPosition16 Convert(TokenMovePosition to) const {
		TokenPosition16 t;
		t.assign(begin(),end());

		iterator i=std::find(t.begin(),t.end(),16);
		if(i==t.end())
		throw AllError("16 not found");

		int bak=t[to];
		t[to]=*i;
		*i=bak;

		return t;
	}

	TokenFree GetTokenFree() const {
		const_iterator i=std::find(begin(),end(),16);
		if(i==end())
			throw AllError("16 not found");
		TokenFree tf;
		tf.reserve(4);
		size_t pos=std::distance(begin(),i);
		// move up
		if(pos>3)
			tf.push_back(pos-4);
		// move down
		if(pos<12)
			tf.push_back(pos+4);
		// move left
		if(pos%4!=0)
			tf.push_back(pos-1);
		// move right
		if((pos+1)%4!=0)
			tf.push_back(pos+1);
		return tf;
	}

	bool operator < (const TokenPosition16&t) const {
		const int icount=16;
		if(t.size()!=icount || size()!=icount)
			throw AllError("Wrong tokens");
		int i=0;
		int ret=0;
		while(i<icount && !(ret=at(i)-t.at(i))) {
			i++;
		};
		return ret<0;
	}
};

std::ostream& operator << (std::ostream &o,const TokenPosition16& t) {
	for(int i=0;i<t.size();i++) {
		if(i%4==0)
			o<<std::endl;
		o.width(3);
		if(t[i]==16)
			o<<" ";
		else
			o<<t[i];
	}
	return o;
};

typedef TokenPosition16 TokenPosition;

typedef std::set<TokenPosition> TokenSet;

typedef std::vector<const TokenPosition*> TokenExpand;

struct Vertex {
	const Vertex* parent;
	int d;
	const TokenPosition* token;

	Vertex(const TokenPosition* t,const Vertex* p,int d):token(t),parent(p),d(d){}
	Vertex(const TokenPosition* t):token(t){}

	bool operator < (const Vertex&v) const {
		return token<v.token;
	}
};

typedef std::set<Vertex> VertexSet;

class VertexOpenList:public std::multimap<int,const Vertex*> {
	std::map<const Vertex*,std::multimap<int,const Vertex*>::iterator> m_index;
public:
	typedef std::multimap<int,const Vertex*> F2V;
	typedef std::map<const Vertex*,std::multimap<int,const Vertex*>::iterator> V2F;

	F2V::iterator push(int f,const Vertex* i) {
		F2V::iterator p=F2V::insert(std::make_pair(f,i));
		m_index.insert(std::make_pair(i,p));
		return p;
	}
 
	const Vertex* pop() {
		F2V::iterator i=F2V::begin();
		const Vertex* o=i->second;
		F2V::erase(i);
		m_index.erase(o);
		return o;
	}

	F2V::iterator find(const Vertex* i) {
		V2F::iterator v=m_index.find(i);
		if(v==m_index.end())
		return F2V::end();
		else
		return v->second;
	}
};

class GraphExtender:public TokenSet {
	TokenPosition m_goal;

public:
	GraphExtender() {
	}

	TokenSet::iterator AddVertex(const TokenPosition &tp) {
		TokenSet::iterator its=insert(tp).first;
		return its;
	}

	void SetGoal(const TokenPosition &tp) {
		m_goal=tp;
	}

	TokenExpand Extend(const TokenPosition* t)
	{
		TokenExpand te;
		TokenFree tf=t->GetTokenFree();
		for(TokenFree::iterator i=tf.begin();i!=tf.end();i++) {
			TokenPosition tp=t->Convert(*i);
			TokenSet::const_iterator its=insert(tp).first;
			te.push_back(&*its);
		}
		return te;
	}

	int Heuristic(const TokenPosition* i) {
		if(m_goal.empty())
		throw AllError("no goal");
		return i->Calc(m_goal);
	}

	bool CheckGoal(const TokenPosition* i) {
		if(m_goal.empty())
			throw AllError("no goal");
		return i->Goal(m_goal);
	}
};

class AStarSearch:public GraphExtender,VertexOpenList,public VertexSet {
public:
	const Vertex* AddVertex(const TokenPosition &tp) {
		TokenSet::iterator i=GraphExtender::AddVertex(tp);
		VertexSet::iterator t=VertexSet::insert(Vertex(&*i,0,0)).first;
		return &*t;
	}

	const Vertex* Search(const Vertex* s) {
		VertexOpenList::push(Heuristic(s->token),s);
		if(CheckGoal(s->token))
			return s;
		while(!VertexOpenList::empty()) {
			const Vertex* p=VertexOpenList::pop();
			TokenExpand te=Extend(p->token);
			for(TokenExpand::iterator i=te.begin();i!=te.end();i++) {
				const int w=1; // w(p,t)

				VertexOpenList::iterator o;
				VertexSet::iterator t=VertexSet::find(Vertex(*i));
				if(t==VertexSet::end()) { // same as VertexOpenList::find(t)==end() 
					t=VertexSet::insert(Vertex(*i,&*p,p->d+w)).first;
					o=VertexOpenList::push(t->d+Heuristic(t->token),&*t);
					if(CheckGoal(t->token))
					return &*t;
				} else {
					if(p->d+w<t->d) {
						int d=p->d+w;
						VertexOpenList::iterator o=VertexOpenList::find(&*t);
						if(o!=VertexOpenList::end())
							VertexOpenList::erase(o);
						o=VertexOpenList::push(d+Heuristic(t->token),&*t);
					}
				}
			}
		}
	}
};

int main(int argc, char **argv) {
	try
	{
		TokenPosition initial;
		initial.Random();
		TokenPosition goal;
		goal.Reset();

		std::cout<<"Initial Token:"<<std::endl;
		std::cout<<initial<<std::endl;
		
		AStarSearch s;
		cout << "astar finished" << endl;
		s.SetGoal(goal);
		const Vertex* i=s.Search(s.AddVertex(initial));

		TokenExpand te;
		for(;i->parent!=0;i=i->parent) {
			te.push_back(i->token);
		}

		std::cout<<std::endl;
		std::cout<<"Walk count:"<<te.size()<<std::endl;
		std::cout<<"Number of vertex:"<<((VertexSet&)s).size()<<std::endl;

		for(TokenExpand::reverse_iterator i=te.rbegin();i!=te.rend();i++) {
			std::cout<<**i<<std::endl;
		}

		std::cout<<std::endl<<"Done!"<<std::endl<<std::endl;
	} catch(std::exception&e) {
		std::cerr<<"Exception: "<<e.what()<<std::endl;
	}
}




