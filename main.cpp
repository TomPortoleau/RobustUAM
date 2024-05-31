
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <ilcp/cp.h>
#include <cmath>
#include <time.h> 

using namespace std;


//================================================== utilitary functions

struct Instance {
    int nbFlights, nbNodes, nbConf;
    vector<vector<float > > dist;
    vector<vector<float > > speed;
    vector<vector<int > > flights;
    map< vector<int>, float > conflicts;

};

struct Scenario {
    int nbFlights, budget;
    vector<float> scenario;
};

struct Solution {
    vector<vector<float> > strategic;
};

void display_vector(vector<int> v) {
    for (int i = 0; i < v.size(); ++i)
    {
        cout << v[i] << " ";
    }
    cout << endl;
}

void display_vector_f(vector<float> v) {
    for (int i = 0; i < v.size(); ++i)
    {
        cout << v[i] << " ";
    }
    cout << endl;
}

std::vector<std::string> split(const std::string& s, char delim) {
    std::vector<std::string> result;
    std::stringstream ss(s);
    std::string item;

    while (getline(ss, item, delim)) {
        result.push_back(item);
    }

    return result;
}

int get_index_intvector(vector<int> v, int e) {
    if (v.size() == 0) {
        return -1;
    }
    auto it = find(v.begin(), v.end(), e);
    if (it != v.end()) {
        return it - v.begin();
    }
    else {
        return -1;
    }
}

//================================================== instances and scenarios functions

Instance readFile(string filename) {
    string tmp;
    vector<string> splitString;
    int index, n1, n2, node;
    float tau;
    ifstream file; file.open(filename);
    Instance inst;
    file >> inst.nbNodes >> inst.nbFlights >> inst.nbConf;
    //cout << inst.nbNodes << " " << inst.nbFlights;
    inst.dist.resize(inst.nbNodes);
    for (int i = 0; i < inst.nbNodes; i++) {
        inst.dist[i].resize(inst.nbNodes);
        for (int j = 0; j < inst.nbNodes; j++) {
            file >> inst.dist[i][j];
        }
    }

    inst.speed.resize(inst.nbNodes);
    for (int i = 0; i < inst.nbNodes; i++) {
        inst.speed[i].resize(inst.nbNodes);
        for (int j = 0; j < inst.nbNodes; j++) {
            file >> inst.speed[i][j];
        }
    }

    getline(file, tmp);
    inst.flights.resize(inst.nbFlights);
    for (int i = 0; i < inst.nbFlights; i++) {
        getline(file, tmp);
        splitString = split(tmp, ' ');
        for (int j = 1; j < splitString.size(); j++) {
            inst.flights[i].push_back(stoi(splitString[j]));
        }
    }


    for (int i = 0; i < inst.nbConf; i++) {
        file >> index >> n1 >> n2 >> node >> tau;
        inst.conflicts[{ n1, n2, node }] = tau;
    }

    return inst;
}

Scenario initScenario(int nbFlight, int budget) {
    Scenario initial;
    initial.nbFlights = nbFlight;
    initial.budget = budget;
    initial.scenario.resize(nbFlight);
    for (int i = 0; i < initial.scenario.size(); i++)
    {
        initial.scenario[i] = 0;
    }
    return initial;
}

Scenario testScenario(int nbFlight, int budget) { //for testing purpose
    Scenario initial;
    initial.nbFlights = nbFlight;
    initial.budget = budget;
    initial.scenario.resize(nbFlight);
    for (int i = 0; i < initial.scenario.size(); i++)
    {
        initial.scenario[i] = 1;
    }
    return initial;
}

Scenario evenScenario(int nbFlight, int budget) { //returns a scenario with evenly shared budget
    Scenario initial;
    initial.nbFlights = nbFlight;
    initial.budget = budget;
    initial.scenario.resize(nbFlight);
    for (int i = 0; i < initial.scenario.size(); i++)
    {
        initial.scenario[i] = float(budget)/nbFlight;
    }
    return initial;
}

Scenario initScenario(int nbFlight, int budget, vector<float> scenario) {
    Scenario initial;
    initial.nbFlights = nbFlight;
    initial.budget = budget;
    initial.scenario.resize(nbFlight);
    for (int i = 0; i < initial.scenario.size(); i++)
    {
        initial.scenario[i] = scenario[i];
    }
    return initial;
}
//================================================== benders

float BendersAdversSubRoutine(Instance inst, Scenario scen, Solution sol) {
    float res = 0;
    IloEnv env;
    IloModel model(env);
    //variables
    vector<IloIntervalVarArray> tix;
    tix.resize(inst.nbFlights);

        for (int i = 0; i < tix.size(); i++)
        {
            tix[i] = IloIntervalVarArray(env, inst.flights[i].size());
            for (int x = 0; x < inst.flights[i].size(); x++) {
                tix[i][x] = IloIntervalVar(env);
                tix[i][x].setLengthMin(1);
                tix[i][x].setLengthMax(1);
            }
        }

    //constraints

    for (int i = 0; i < inst.nbFlights; i++) //precedence for flights tix
    {
        for (int x = 1; x < inst.flights[i].size(); x++)
        {
            model.add(IloStartOf(tix[i][x]) == IloStartOf(tix[i][x - 1]) + inst.dist[inst.flights[i][x]][inst.flights[i][x - 1]] / inst.speed[inst.flights[i][x]][inst.flights[i][x - 1]]);
        }
    }

    map< vector<int>, float >::iterator it;
    for (it = inst.conflicts.begin(); it != inst.conflicts.end(); it++) { //constraints for conflicts for tix variables
        int i = it->first[0];
        int j = it->first[1];
        int x = it->first[2];
        int x_index_i = get_index_intvector(inst.flights[i], x);
        int x_index_j = get_index_intvector(inst.flights[j], x);
        model.add(IloStartOf(tix[i][x_index_i]) - IloEndOf(tix[j][x_index_j]) >= it->second || IloStartOf(tix[j][x_index_j]) - IloEndOf(tix[i][x_index_i]) >= it->second);
    }

    for (int i = 0; i < inst.nbFlights; i++) //constraints coupling hatix and tix
    {
        model.add( IloStartOf(tix[i][0])  >= sol.strategic[i][0]  + scen.scenario[i]); //CONTINUE THE WORK HERE, SOLUTION NOT STORED
       
    }
    
    //objective
        IloExpr expr(env);
        for (int i = 0; i < inst.nbFlights; i++) {
            expr += IloEndOf(tix[i][0]);
        }
    IloObjective objective = IloMinimize(env, expr);
    model.add(objective);
    //solving
    IloCP cp(model);
    cp.setParameter(IloCP::LogVerbosity, IloCP::Quiet);
    cp.setParameter(IloCP::TimeLimit, 15);
    cp.setParameter(IloCP::TimeMode, IloCP::ElapsedTime);

    cp.solve();
   
    res = cp.getObjValue();
    env.end();
    return res;

}

pair<Scenario, float> bendersAdvers(Instance inst, int budget, Solution sol) { //the local search
    cout << "========================STARTING ADVERSARIAL" << endl;
    clock_t start = clock();
    clock_t end;
    int time_limit = 5;
    int elapsed_time = 0;
    int G = 10;
    Scenario currScen = evenScenario(inst.nbFlights, budget);
    Scenario bestScen = currScen;
    float currObj = BendersAdversSubRoutine(inst, currScen, sol);
    float bestObj = currObj;
    int i,nbEl, el;
    float g;
    vector<int> F;
    vector<float> tmp_scen;
    vector<float> scenarioK;
    while (elapsed_time<time_limit) {
        vector<Scenario> N;
        N.resize(0);
        for (int k = 0; k < log(0.1)/(log(float(inst.nbFlights-1)/inst.nbFlights)); k++)
        {
            i = rand() % inst.nbFlights;
            g = min((float(budget)/G)*(rand() /(float)RAND_MAX + 1), currScen.scenario[i]);
            nbEl = rand() % (inst.nbFlights/2) + 1;
            F.resize(0);
            while (F.size() <= nbEl) {
                el = rand() % inst.nbFlights;
                if (get_index_intvector(F, el) == -1 && el != i) {
                    F.push_back(el);
                }
            }
            scenarioK = currScen.scenario;
            scenarioK[i] = currScen.scenario[i] - g;
            for (int e = 0; e < inst.nbFlights; e++)
            {
                if (get_index_intvector(F, e) != -1 && e != i) {
                    scenarioK[e] = currScen.scenario[e] + float(g) / F.size();
                }
            }
          
            N.push_back(initScenario(inst.nbFlights, budget, scenarioK));
        }
        /*cout << "Test N";
        
        for (int k = 0; k < N.size(); k++) {
            float sum = 0;
            for (int i = 0; i < N[k].scenario.size(); i++)
            {
                sum += N[k].scenario[i];
            }
            display_vector_f(N[k].scenario);
            cout << sum << endl;
        }*/
        bool flag = false;
        for (int k = 0; k < N.size(); k++)
        {
            currObj = BendersAdversSubRoutine(inst, N[k], sol);
            if (currObj >= bestObj) {
                bestObj = currObj;
                //memcpy(&currScen, &bestScen, sizeof(bestScen));
                bestScen = initScenario(inst.nbFlights, budget, N[k].scenario);
                flag = true;
                //cout << "new best objective in ADV : " << bestObj<<endl;
            }
        }
        if (not flag) {
            currScen = evenScenario(inst.nbFlights, budget);
            currObj = BendersAdversSubRoutine(inst, currScen, sol);
        }
        end = clock();
        elapsed_time = (int)(end - start) / CLOCKS_PER_SEC;
        //cout << "curr time : " << elapsed_time << endl;
    }
    
    return make_pair(bestScen, bestObj);
}

pair<Solution, float> bendersMaster(Instance inst, vector<Scenario> U) {
    cout << "==========================SOLVING MASTER" << endl;
    float res = 0;
    IloEnv env;
    IloModel model(env);
    //variables
    vector<IloIntervalVarArray> tix;
    vector<vector<IloIntervalVarArray> > hatix;
    hatix.resize(U.size());
    tix.resize(inst.nbFlights);
    for (int l = 0; l < U.size(); l++)
    {
        hatix[l].resize(inst.nbFlights);
    }
    
    for (int l = 0; l < U.size(); l++) {
        for (int i = 0; i < tix.size(); i++)
        {
            tix[i] = IloIntervalVarArray(env, inst.flights[i].size());
            hatix[l][i] = IloIntervalVarArray(env, inst.flights[i].size());
            for (int x = 0; x < inst.flights[i].size(); x++) {
                tix[i][x] = IloIntervalVar(env);
                tix[i][x].setLengthMin(1);
                tix[i][x].setLengthMax(1);
                hatix[l][i][x] = IloIntervalVar(env);
                hatix[l][i][x].setLengthMin(1);
                hatix[l][i][x].setLengthMax(1);
            }

        }
    }

    IloNumVarArray zl(env, U.size());
    for (int l = 0; l < U.size(); l++) {
        zl[l] = IloNumVar(env);
    }
    IloNumVar z(env);
    //constraints

    for (int l = 0; l < U.size(); l++) //objective related constraints
    {
        model.add(z >= zl[l]);
    }

    for (int l = 0; l < U.size(); l++) { 
        IloExpr expr(env);
        for (int i = 0; i < inst.nbFlights; i++) {
            expr += IloEndOf(hatix[l][i][0]);
        }
        model.add(zl[l] >= expr);
    }

    for (int i = 0; i < inst.nbFlights; i++) //precedence for flights tix
    {
        for (int x = 1; x < inst.flights[i].size(); x++)
        {
            model.add(IloStartOf(tix[i][x]) == IloStartOf(tix[i][x - 1]) + inst.dist[inst.flights[i][x]][inst.flights[i][x - 1]] / inst.speed[inst.flights[i][x]][inst.flights[i][x - 1]]);
        }
    }

    for (int l = 0; l < U.size(); l++) { //precedence for flights hatix
        for (int i = 0; i < inst.nbFlights; i++) 
        {
            for (int x = 1; x < inst.flights[i].size(); x++)
            {
                model.add(IloStartOf(hatix[l][i][x]) == IloStartOf(hatix[l][i][x - 1]) + inst.dist[inst.flights[i][x]][inst.flights[i][x - 1]] / inst.speed[inst.flights[i][x]][inst.flights[i][x - 1]]);
            }
        }
    }

    map< vector<int>, float >::iterator it;
    for (it = inst.conflicts.begin(); it != inst.conflicts.end(); it++) { //constraints for conflicts for tix variables
        int i = it->first[0];
        int j = it->first[1];
        int x = it->first[2];
        int x_index_i = get_index_intvector(inst.flights[i], x);
        int x_index_j = get_index_intvector(inst.flights[j], x);
        model.add(IloStartOf(tix[i][x_index_i]) - IloEndOf(tix[j][x_index_j]) >= it->second || IloStartOf(tix[j][x_index_j]) - IloEndOf(tix[i][x_index_i]) >= it->second);
    }

    for (int l = 0; l < U.size(); l++) {
        for (it = inst.conflicts.begin(); it != inst.conflicts.end(); it++) { //constraints for conflicts for hatix variables
            int i = it->first[0];
            int j = it->first[1];
            int x = it->first[2];
            int x_index_i = get_index_intvector(inst.flights[i], x);
            int x_index_j = get_index_intvector(inst.flights[j], x);
            model.add(IloStartOf(hatix[l][i][x_index_i]) - IloEndOf(hatix[l][j][x_index_j]) >= it->second || IloStartOf(hatix[l][j][x_index_j]) - IloEndOf(hatix[l][i][x_index_i]) >= it->second);
        }
    }

    for (int i = 0; i < inst.nbFlights; i++) //constraints coupling hatix and tix
    {
        for (int l = 0; l < U.size(); l++) {
            model.add(IloStartOf(hatix[l][i][0]) >= IloStartOf(tix[i][0]) + U[l].scenario[i]);
        }
    }


    //objective
    IloObjective objective = IloMinimize(env, z);
    model.add(objective);
    //solving
    IloCP cp(model);
    cp.setParameter(IloCP::LogVerbosity, IloCP::Terse);
    cp.setParameter(IloCP::TimeLimit, 15);
    cp.setParameter(IloCP::TimeMode, IloCP::ElapsedTime);

    cp.solve();
    Solution sol;
    sol.strategic.resize(tix.size());
    for (int i = 0; i < sol.strategic.size(); i++) {
        sol.strategic[i].resize(tix[i].getSize());
        for (int x = 0; x < sol.strategic[i].size(); x++) {
            sol.strategic[i][x] = cp.getValue(IloStartOf(tix[i][x]));
        }
    }
    res = cp.getObjValue();
    env.end();
    return make_pair(sol, res);
} 


float bendersMain(Instance inst, int budget) {
    bool flag = true;
    vector<Scenario> U;
    pair<Solution, float> res;
    U.resize(0);
    U.push_back(initScenario(inst.nbFlights, budget));
    while (flag) {
        res = bendersMaster(inst, U);
        pair<Scenario, float> newres = bendersAdvers(inst, budget, res.first);
        if (newres.second > res.second) {
            U.push_back(newres.first);
            //cout << "=====================new master is " << newres.second << ", old was :"<< res.second << endl;
            /*for (int i = 0; i < U.size(); i++)
            {
                display_vector_f(U[i].scenario);
            }*/
            flag = true;
        }
        else {
            flag = false;
        }
    }
    
    return res.second;
}



//================================================== main

int main()
{
    Instance inst = readFile("testgrid_0.tsruami");
    srand(1);
    int budget = 15;
    bendersMain(inst, budget);
    

    return 0;
}
