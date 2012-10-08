#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <ctime>
#include <algorithm>
#include <map>
#include <set>
#include <functional>
#include <numeric>
#include <utility>

#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <cstring>
using namespace std;
typedef vector<int> vi; 
typedef vector<vi> vvi; 
typedef vector<double> vd; 
typedef vector<vector<double> > vvd; 
typedef pair<int,int> ii; 
typedef pair<char,char> cc; 
typedef pair<ii,char> Robot;


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
    map< char, ii > diffs2;
    map< cc , string > moves;
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
		
		diffs2['^'] = ii(-1,0);
		diffs2['v'] = ii(1,0);
		diffs2['>'] = ii(0,1);
		diffs2['<'] = ii(0,-1);
		
		moves[cc('N','^')] = 'F';
		moves[cc('N','>')] = "RF";
		moves[cc('N','<')] = "LF";
		moves[cc('N','v')] = "LLF";
		
		
		moves[cc('E','^')] = "LF";
		moves[cc('E','>')] = 'F';
		moves[cc('E','<')] = "RRF";
		moves[cc('E','v')] = "RF";
		
		
		moves[cc('S','^')] = "LLF";
		moves[cc('S','>')] = "LF";
		moves[cc('S','<')] = "RF";
		moves[cc('S','v')] = "F";
		
		
		moves[cc('W','^')] = "RF";
		moves[cc('W','>')] = "RRF";
		moves[cc('W','<')] = 'F';
		moves[cc('W','v')] = "LF";
	}
	
	double heuristic(ii p1, ii p2)
	{
		return ((p1.first-p2.first)*(p1.first-p2.first) + (p1.second-p2.second)*(p1.second-p2.second));
	}
	static bool const myfunction (const vector<double>& open1, const vector<double>& open2) {
			return (open1[0]< open2[0]);
	}
	void turn(vector<Robot>& Robots, char direction)
    {
		tr(Robots, it)
		{
			it->second = turns[cc(it->second,direction)];
		}
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
	vector<Robot> forward(vector<Robot>& Robots)
	{
		//cout<<"new forward";
		int m,i;
		//cout<<Robots.size();
		//if(Robots.size() == 22) exit(0);
		
		//if(Robots.size() == 22) exit(0);
		tr(Robots, it)
		{
			//cout<<Robots[0].direction;
			ii diff = diffs[it->second];
			m = it->first.first + diff.first;
			i = it->first.second + diff.second;
			if(m<R && i<C && m>-1 && i>-1 && !is_hitting_wall(ii(m,i)))
			{
				it->first.first = m;
				it->first.second = i;
				if(is_on_exit(ii(m,i)))
					it->second = 'X';
			}
		}
		vector<Robot> r1;
		tr(Robots, it)
		{
			if(it->second != 'X')
			{
				r1.pb(*it);
			}
		}
		return r1;
		
		
	}
	void locomote(vector<Robot>& Robots, char command)
	{
		if(command == 'L' || command == 'R')
		{
			turn(Robots, command);
		}
		else
		{
			Robots = forward(Robots);
		}
	}
	void move(vector<Robot>& Robots, string locomotions)
	{
		for(int i=0;i<(int)locomotions.size();i++)
		{
			locomote(Robots, locomotions[i]);
		}
	}
	string astar(const vvi& map, ii init, ii goal, vector<Robot>& Robots,  vector<Robot>::iterator Robotidx)
	{
		
		vector<ii> delta;
		delta.pb(ii(-1, 0 )); //# go up
		delta.pb(ii(0, -1)); //# go left
		delta.pb(ii(1, 0 )); //# go down
		delta.pb(ii(0, 1 )); //# go right

		char delta_name[] = {'^', '<', 'v', '>'};

		int cost = 1;
		vvi closed(R, vector<int>(C, 0));
		closed[init.first][init.second] = 1;
		vvi expand(R, vector<int>(C, 0));
		vvi action(R, vector<int>(C, -1));
		double x = init.first;
		double y = init.second;
		double g = 0;
		double h = heuristic(init,goal);
		double f = g + h;
		double open1[] = {f, g, h, x, y};
		vvd open;
		vector<double> ttemp;
		ttemp.assign(open1, open1 + 5);
		open.pb(ttemp);

		bool found = false; // flag that is set when search is complet
		bool resign = false; // flag set if we can't find expand
		int count = 0;
		
		while(!found && !resign)
		{
			if(open.size() == 0)
				resign = true;
			else
			{
				sort(open.begin(), open.end(),myfunction);
				vd next = open.front();
				open.erase(open.begin());
			    x = (int)next[3];
				y = (int)next[4];
				g = next[1];
				expand[x][y] = count;
				count += 1;
				if(x == goal.first && y == goal.second)
					found = true;
				else
				{
					for(int i=0;i<(int)delta.size();i++)
					{
						int x2 = x + delta[i].first;
						int y2 = y + delta[i].second;
						int g2,f2,h2;
						if( x2 >= 0 && x2 < R && y2 >= 0 && y2 < C )
						{
							if( closed[x2][y2] == 0 && map[x2][y2] == 0 )
							{
								g2 = g + cost;
								h2 = heuristic(ii(x2,y2),goal);
								f2 = g2 + h2;
								double open1[] = {f2, g2, h2, x2, y2};
								vector<double> ttemp;
								ttemp.assign(open1, open1 + 5);
								open.pb(ttemp);
								closed[x2][y2] = 1;
								action[x2][y2] = i;
							}
						}
					}
                }
			}
		}	
		
		/*for(int i=0;i<R;i++)
		{
			for(int j=0;j<C;j++)
			{
				if(expand[i][j]<10)
					cout<<expand[i][j]<<"   ";
				else if(expand[i][j]<100)
					cout<<expand[i][j]<<"  ";
				else
					cout<<expand[i][j]<<" ";
			}
			cout<<endl;
		}*/
		vector< vector<char> > policy(R, vector<char>(C, '#'));
		x = goal.first;
		y = goal.second;
		int x2,y2;
		policy[x][y] = '*';
		while(x != init.first || y != init.second)
		{	x2 = x - delta[action[x][y]].first;
			y2 = y - delta[action[x][y]].second;
			policy[x2][y2] = delta_name[action[x][y]];
			x = x2;
			y = y2;
			//cout<<x<<" "<<y<<endl;
		}
		/*for(int i=0;i<R;i++)
		{
			for(int j=0;j<C;j++)
			{
				cout<<policy[i][j]<<"";
			}
			cout<<endl;
		}
		cout<<endl;
		cout<<endl;*/
		int a = init.first;
		int b = init.second;
		/*cout<<"Init "<<init.first<<" "<<init.second<<endl;
		cout<<"Goal "<<goal.first<<" "<<goal.second<<endl;*/
		string output = "";
		while(true)
		{	
			string locomotions = moves[cc(Robotidx->second,policy[a][b])];//what if a robot gets deleted
			output.append(locomotions);
			move(Robots,locomotions);
			/*cout<<policy[a][b]<<" ";
			cout<<a<<" "<<b<<" "<<endl;
			//cout<<locomotions<<endl;
			//display(Robots,R,C);
			//cout<<endl;
			//cout<<endl;
			cout<<a<<" "<<b<<" ";
			cout<<diffs2[policy[a][b]].first<<" "<<diffs2[policy[a][b]].second<<" ";
			cout<<endl<<b<<"+"<<diffs2[policy[a][b]].second<<"=";*/
			if(policy[a][b] == '*') break;
			x2 = a;
			y2 = b;
			a = a + diffs2[policy[x2][y2]].first;
			b = b + diffs2[policy[x2][y2]].second;
			
			/*cout<<b<<endl;
			cout<<a<<" "<<b<<" ";*/
		}
		return output;
	}
	void min_dist(const vector<Robot> &Robots, int &p, int &m)
	{
		double diff=999999.0;
		p = 0,m = 0;
		double_t temp;
		for(int i=0;i<(int)exits.size();i++)
		{
			for(int j=0;j<(int)Robots.size();j++)
			{
				temp = heuristic(ii(Robots[j].first.first,Robots[j].first.second), exits[i]);
				if(temp<diff)
				{ 
					diff = temp;
					p = i;
					m = j;
				}
			}
			
		}
		
	}
	void display(vector<Robot>& Robots,int R, int C)
	{
		vector< vector<char> > map(R, vector<char>(C, '.'));
		tr(walls,it)
		{
			map[it->first][it->second] = '#';
		}
		tr(exits,it)
		{
			map[it->first][it->second] = 'x';
		}
		tr(Robots,it)
		{
			map[it->first.first][it->first.second] = it->second;
		}
		for(int i=0;i<R;i++)
		{
			for(int j=0;j<C;j++)
			{
				cout<<map[i][j]<<"";
				
			}
			cout<<endl;
		}
	}
    string evacuateAll(vector <string> cave, double V)
    {
		vector<Robot> Robots;
		
//-------------------EXTRACT WALLS, EXITS AND ROBOTS----------------------------------//
		for(int m=0;m<(int)cave.size();m++)
		{
			for(int i=0;i<(int)cave[m].size();i++)
			{
				if(cave[m][i] == 'E' || cave[m][i] == 'S' || cave[m][i] == 'W' || cave[m][i] == 'N')
				{
					Robot temp = Robot(ii(m,i),cave[m][i]);
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
		vvi map(R, vector<int>(C, 0));
		for(int i=0;i<(int)walls.size();i++)
		{
			map[walls[i].first][walls[i].second] = 1;
			 
		}
		int m,p;
		string output = "";
		while(Robots.size() != 0)
		{
			min_dist(Robots, p, m);
			/*cout<<m<<" "<<p<<endl;
			cout<<"Robot:"<<Robots[m].x<<" "<<Robots[m].y<<endl;
			cout<<"exit:"<<exits[p].first<<" "<<exits[p].second<<endl;
			*/
			//int h=0;
			/*tr(Robots,it)
			{
				cout<<h<<" "<<it->x<<" "<<it->y<<" "<<it->direction<<endl;
				h++;
			}
			display(Robots,R,C);*/
			
			//cout<<"Sizze:"<<Robots.size()<<endl;
			output.append(astar(map, ii(Robots[m].first.first,Robots[m].first.second), exits[p], Robots,Robots.begin()+m));
			/*display(Robots,R,C);
			h=0;
			tr(Robots,it)
			{
				cout<<h<<" "<<it->x<<" "<<it->y<<" "<<it->direction<<endl;
				h++;
			}*/
			//cout<<"Sizze:"<<Robots.size()<<endl;
		}
		return output;
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

    
