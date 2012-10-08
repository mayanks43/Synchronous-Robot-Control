#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <ctime>
#include <algorithm>
#include <list>
#include <map>
#include <set>
#include <deque>
#include <queue>
#include <stack>
#include <bitset>
#include <functional>
#include <numeric>
#include <utility>
#include <iomanip>
#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <cstring>
using namespace std;
typedef vector<int> vi; 
typedef vector<vi> vvi; 
typedef vector<vector<double> > vvd; 
typedef pair<int,int> ii; 

typedef pair<ii,char> Robot; 
typedef pair<char,char> cc; 
typedef pair< vector<Robot>, pair<vvd,int> > state;
typedef char action;
typedef double cost;
typedef pair<state,cost> psc;
typedef map<action, psc > successors;
typedef pair<action,cost> pac;
typedef pair< vector<pac>, vector<state> > path;


#define sz(a) int((a).size()) 
#define pb push_back 
#define all(c) (c).begin(),(c).end() 
#define tr(container, it) \
      for(typeof(container.begin()) it = container.begin(); it != container.end(); it++) 
#define present(c,x) ((c).find(x) != (c).end()) 
#define cpresent(c,x) (find(all(c),x) != (c).end()) 

class SynchronousControl
{
	
    
    vector<ii> exits;
    vector<ii> walls;
    map< cc , char > turns;
    map< char, ii > diffs;
    int R,C;
    public: 
    SynchronousControl()
    {
		turns[cc('N','L')] = 'W';
		turns[cc('S','L')] = 'E';
		turns[cc('E','L')] = 'N';
		turns[cc('W','L')] = 'S';
		turns[cc('N','R')] = 'E';
		turns[cc('S','R')] = 'W';
		turns[cc('E','R')] = 'S';
		turns[cc('W','R')] = 'N';
		diffs['N'] = ii(-1,0);
		diffs['S'] = ii(1,0);
		diffs['E'] = ii(0,1);
		diffs['W'] = ii(0,-1);
	}
    psc turn(const state& s, char direction)
    {
		vector<Robot> Robots;
		tr(s.first, it)
		{
			Robot temp = make_pair(it->first, turns[cc(it->second,direction)]);
			Robots.pb(temp);
		}
		state p = state(Robots,s.second);
		return psc(p,1.0);
	}
	bool is_hitting_wall(ii position)
	{
		tr(walls,it)
		{
			if(it->first == position.first && it->second == position.second)
			{
				return true;
			}
		}
		return false;
	}
	bool is_on_exit(ii position)
	{
		tr(exits,it)
		{
			if(it->first == position.first && it->second == position.second)
			{
				return true;
			}
		}
		return false;
	}
	psc forward(const state& s,double V)
	{
		
		vector<Robot> Robots;
		vector<ii> cells;
		int m,i;
		double weight;
		
		vector< vector<double> > new_contamination(R, vector<double>(C));
		for(int k=0;k<R;k++)
		{
			for(int l=0;l<C;l++)
			{
				new_contamination[k][l] = s.second.first[k][l];
			}
		} 
		tr(s.first, it)
		{
			Robot temp;
			ii diff = diffs[it->second];
			m = it->first.first + diff.first;
			i = it->first.second + diff.second;
			if(m<R && i<C && m>-1 && i>-1 && !is_hitting_wall(ii(m,i)))
			{
				temp = Robot(ii(m,i),it->second);
				cells.pb(ii(m,i));
				if(!is_on_exit(ii(m,i)))
					Robots.pb(temp);
			}
			else
			{
				temp = Robot(it->first,it->second);
				Robots.pb(temp);
			}
			
		}
		if(cells.empty())
		{
			cout<<endl<<"Empty"<<endl;
			weight = 1.0;
		}
		else
		{
			double max = 0.0;
			tr(cells,it)
			{
				if(s.second.first[it->first][it->second] > max)
					max = s.second.first[it->first][it->second];
				new_contamination[it->first][it->second] = s.second.first[it->first][it->second]+1;
			}
			weight = 1 + log (1 + max) / V;
		}
		
		state p = state(Robots,pair<vvd,int>(new_contamination,Robots.size()));
		return psc(p,weight);
	}
    state final_state(const path& p1)
	{
		return p1.second.back();
	}
	bool is_goal(const state& s)
	{
		if(s.first.empty())
		{
			return true;
		}
		else return false;
	}
	bool equal_Robot(Robot r1, Robot r2)
	{
		if(r1.first.first==r2.first.first &&  r1.first.second==r2.first.second &&  r1.second==r2.second)
			return true;
		else return false;
	}
	bool equal(const vector<Robot>& r1, const vector<Robot>& r2)
	{
		
		if (r1.size() != r2.size())
			return false;
		else
		{
			tr(r1,it1)
			{
				int found = false;
				tr(r2,it2)
				{
					if(equal_Robot(*it1, *it2))
					{
						found = true;
						break;
					}
				}
				if(found == false)
					return false;
			}
		}
		
		return true;
	}
	void add(vector<state>& set_of_states, const state& s)
	{
		
		tr(set_of_states,it)
		{
			if(equal(it->first,s.first))
				return;
		}
		set_of_states.pb(s);
	}
	bool not_in(vector<state>& set_of_states, const state& s)
	{
		tr(set_of_states,it)
		{
			if(equal(it->first,s.first))
				return false;
		}
		return true;
	}
    successors rsuccessors(state& current_state,double V)
    {
		successors m;
		psc left = turn(current_state, 'L');
		psc right = turn(current_state, 'R');
		psc ahead = forward(current_state,V);
		m['L'] = left;
		m['R'] = right;
		m['F'] = ahead;
		return m;
	}
	static cost path_cost(const vector<pac>& pac_list)
	{
		return pac_list[pac_list.size()-1].second;
	}
	static int number_of_cars(const vector<state>& state_list)
	{
		return state_list[state_list.size()-1].second.second;
	}
	static bool const myfunction (const path& path1, const path& path2) {
		//cout<<"asdsa"<<path1.second.back().first.size()<<" "<<path2.second.back().first.size()<<endl;
		
		if(number_of_cars(path1.second) < number_of_cars(path2.second))
		{
			
			return true;
		}
		else if(number_of_cars(path1.second) == number_of_cars(path2.second))
		{
			if(path_cost(path1.first) < path_cost(path2.first))
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		else
		{
			return false;
		}
		
	}

	void add_to_frontier(vector<path>& frontier,path& path2)
	{
		//# Find if there is an old path to the final state of this path.
		int old = -1;
		for(int i=0;i<(int)frontier.size();i++)
		{
			if(final_state(frontier[i]) == final_state(path2))
			{
				old = i;
				break;
			}
		}
		if(old != -1 && path_cost(frontier[old].first) < path_cost(path2.first))
			return; //# Old path was better; do nothing
		else if (old != -1)
			frontier.erase(frontier.begin()+old); //# Old path was worse; delete it
		//## Now add the new path and re-sort
		int n1 = number_of_cars(path2.second);
		
		for(int i=0;i<(int)frontier.size();i++)
		{
			if(n1<number_of_cars(frontier[i].second))
			{
				frontier.clear();
				break;
			}
		}
		frontier.pb(path2);
		sort(frontier.begin(), frontier.end(),myfunction);
	}
	string get_path(const path& p1)
	{
		string commands = "";
		tr(p1.first,it)
		{
			commands.append(1,it->first);
		}
		return commands;
	}
	cost distance(ii p1, ii p2)
	{
		return ((p1.first-p2.first)*(p1.first-p2.first) + (p1.second-p2.second)*(p1.second-p2.second));
	}
	cost heuristic(state& current_state)
	{
		cost diff=999999.0;
		int p = 0,m = 0;
		cost temp;
		for(int i=0;i<(int)exits.size();i++)
		{
			for(int j=0;j<(int)current_state.first.size();j++)
			{
				temp = distance(current_state.first[j].first, exits[i]);
				if(temp<diff)
				{ 
					diff = temp;
					p = i;
					m = j;
				}
			}
			
		}
		return distance(current_state.first[m].first,exits[p]);;
		
	}
	string lowest_cost_search(state& current_state,double V)
	{
		//------------------------THE SEARCH--------------------------//
		
		vector<state> explored;
		vector<path> frontier;
		//init of frontier
		vector<pac> a1;
		a1.pb(pac('\0',0.0));
		vector<state> s1;
		s1.pb(current_state);
		frontier.pb(path(a1,s1));
		while(!frontier.empty())
		{
			//cout<<frontier.size()<<endl;
			path p1 = frontier.front();
			
			//cout<<"Robot number:"<<p1.second.back().first.size()<<endl;
			frontier.erase(frontier.begin());
			//cout<<frontier.size()<<endl;
			state state1 = final_state(p1);
			if(is_goal(state1))
				 return get_path(p1);
			
			add(explored,state1);
			cost p_cost = path_cost(p1.first);
			successors successors1 = rsuccessors(state1,V);
			tr(successors1, iterator)
			{
				if(not_in(explored,iterator->second.first))
				{
					cost total_cost = p_cost + iterator->second.second + heuristic(iterator->second.first);
					path path2 = p1;
					path2.first.pb(pac(iterator->first,total_cost));
					path2.second.pb(iterator->second.first);
					add_to_frontier(frontier, path2);
					//cout<<"Successor Robot number:"<<path2.second.back().first.size()<<endl;
				}
			}
			
		}
		return NULL;
	}
    string evacuateAll(vector <string> cave, double V)
    {
		vector< vector<double> > contamination((int)cave.size(), vector<double>((int)cave[0].size()));
		vector<Robot> Robots;
//-------------------EXTRACT WALLS, EXITS AND ROBOTS----------------------------------//
		for(int m=0;m<(int)cave.size();m++)
		{
			for(int i=0;i<(int)cave[m].size();i++)
			{
				if(cave[m][i] == 'E' || cave[m][i] == 'S' || cave[m][i] == 'W' || cave[m][i] == 'N')
				{
					ii temp1 = make_pair(m,i);
					Robot temp = make_pair(temp1, cave[m][i]);
					Robots.pb(temp);
				}
				else if(cave[m][i] == 'x' )
				{
					ii temp1 = make_pair(m,i);
					exits.pb(temp1);
				}
				else if(cave[m][i] == '#' )
				{
					ii temp1 = make_pair(m,i);
					walls.pb(temp1);
				}
			}
		}
		R = (int)cave.size();
		C = (int)cave[0].size();
		state current_state = state(Robots, pair<vvd,int>(contamination,Robots.size()));
		//string ans = lowest_cost_search(current_state, V);
		
		
		
//------------------------UNIT TESTS---------------------------//		
		/*
		 * 
		 *
		 * 
		 * 
		vector<pac> a1;
		a1.pb(pac('\0',0.0));
		vector<state> s1;
		s1.pb(current_state);
		path bc = path(a1,s1);
		cout<<path_cost(bc.first)<<endl;
		* 
		* 
		* 
		vector<state> explored;
		vector<Robot> copy = Robots;
		copy.pb(Robot(ii(23,24),'S'));
		state copy2 = state(copy, contamination);
		add_set(explored, current_state);
		cout<<"size "<<explored.size()<<endl;
		add_set(explored, copy2);
		cout<<"size "<<explored.size()<<endl;
		add_set(explored, current_state);
		cout<<"size "<<explored.size()<<endl;
		* 
		* 
		* 
		state copy2 = state(Robots, contamination);
		vector<Robot> copy = Robots;
		copy.pb(Robot(ii(23,24),'S'));
		cout<<equal(copy, Robots)<<endl;
		cout<<is_hitting_wall(ii(86,6))<<endl;
		psc p = turn(current_state, 'R');
		tr(p.first.first,it)
		{
			cout<<"Actual Robot:"<<it->first.first<<" "<<it->first.second<<" "<<it->second<<endl;
		}
		psc p = forward(current_state,V);
		successors m = rsuccessors(current_state,V);
		cout<<"Cost:"<<m['F'].second<<endl;
		cout<<"Contamination"<<endl;
		for(int k=0;k<R;k++)
		{
			for(int l=0;l<R;l++)
			{
				cout<<m['F'].first.second[k][l]<<" ";
			}
			cout<<endl;
		}
		cout<<endl;
		tr(m['F'].first.first,it)
		{
			cout<<"Actual Robot:"<<it->first.first<<" "<<it->first.second<<" "<<it->second<<endl;
		}
		successors m = rsuccessors(current_state,V);
		tr(m['L'].first.first,it)
		{
			cout<<"Actual Robot:"<<it->first.first<<" "<<it->first.second<<" "<<it->second<<endl;
		}
		cout<<endl;
		tr(m['R'].first.first,it)
		{
			cout<<"Actual Robot:"<<it->first.first<<" "<<it->first.second<<" "<<it->second<<endl;
		}*/
		return "RLFFFF";
    }  
}; 

int main()
{
    int R;
    SynchronousControl S;
    scanf("%d",&R);
    
    
    vector <string> cave;
    double V;
    string temp;
    for(int m=0;m<R;m++)
    {
		cin>>temp;
		cave.pb(temp);
	}
	
	scanf("%lf",&V);
	cout<<S.evacuateAll(cave,V);
	fflush(stdout);
} 

    
