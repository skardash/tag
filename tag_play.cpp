#include <iostream>
#include <algorithm>
#include <iterator>
#include <vector> 

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>

using namespace std;

struct TerminalOpt {
    TerminalOpt() {
        termios new_settings;
        tcgetattr(0,&stored_settings);
        new_settings = stored_settings;
        new_settings.c_lflag &= (~ICANON);
        new_settings.c_cc[VTIME] = 0;
        new_settings.c_cc[VMIN] = 1;
        tcsetattr(0,TCSANOW,&new_settings);
    }
    ~TerminalOpt() {
        tcsetattr(0,TCSANOW,&stored_settings);
    }
    termios stored_settings;
};

class pos {
public:
	int x; int y;
};

class tag {
public:
	pos empty_pos;
	int field_size;
	vector< vector<int> > field;
	
	tag(int sz) {
		field_size = sz;
		field.resize(sz);
		for (int i=0; i<sz; i++) {
			field[i].resize(sz);
		}
	}
	
	void init() {
		srand(unsigned(time(0)));
		vector<int> myvector;
		int elem_num = field_size*field_size;
		for (int i=0; i<elem_num; ++i) myvector.push_back(i);
		random_shuffle(myvector.begin(), myvector.end());
		for (int i=0; i<field.size(); i++) {
			for (int j=0; j<field[i].size(); j++) {
				field[i][j] = myvector[i*field_size + j];
			}
		}
	}
	
	void show_field() {
		cout << "-----------" << endl;
		for (int i=0; i<field.size(); i++) {
			for (int j=0; j<field[i].size(); j++) {
				if (field[i][j] == 0) {
					cout << "  ";
					empty_pos.x = i; empty_pos.y = j;
				} else cout << field[i][j] << " ";
				if (field[i][j] < 10) cout << " ";
			}
			cout << endl;
		}
		cout << "-----------" << endl;
	}

	bool check(pos side) {	
		int pret_x = empty_pos.x + side.x;
		int pret_y = empty_pos.y + side.y;
		return (pret_x >= 0 && pret_x < field_size && pret_y >= pret_y && pret_y < field_size);
	}	

	void move(pos side) {
		if (check(side)) {
			field[empty_pos.x][empty_pos.y] = field[empty_pos.x + side.x][empty_pos.y + side.y];
			empty_pos.x = empty_pos.x + side.x;
			empty_pos.y = empty_pos.y + side.y;
			field[empty_pos.x][empty_pos.y] = 0;
		}
	}

	void move(int x, int y) {
		pos side; side.x = x; side.y = y;
		move(side);
	}
	
	bool isordered() {
		int curr = 1;
		for (int i=0; i<field.size(); i++) {
			for (int j=0; j<field[i].size(); j++) {
				if (i == field.size()-1 && j == field.size()-1)
					return true;
				if (field[i][j] != curr)
					return false;
				curr++;
			}
		}
		return true;
	}
	
	int getch() {
		struct termios oldt, newt;
		int ch;
		tcgetattr(STDIN_FILENO, &oldt);
		newt = oldt;
		newt.c_lflag &= ~(ICANON | ECHO);
		tcsetattr(STDIN_FILENO, TCSANOW, &newt);
		ch = getchar();
		tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
		return ch;
	}

	void init_not_ordered() {
		do {
			init();
		} while(isordered());
	}

	void start_game() {
		system("clear");		
		init_not_ordered(); 
		show_field();
		while(!isordered()) {	
			int ch = getch();
			if (ch==65) move(1,0); //cout << "up";
			else if (ch==68) move(0,1); //cout << "left";
			else if (ch==67) move(0,-1); //cout << "right";
			else if (ch==66) move(-1,0); // cout << "down";
			else if (ch=='q'-0) break; // cout << "down";
			else if (ch=='r'-0) init(); // cout << "down";
			system("clear");
			show_field();// cout << endl;
		}
		if (isordered())
			cout << "success!" << endl;
	}
};

int main() {
	tag tg(10);
	tg.start_game();
	return 0;
}
