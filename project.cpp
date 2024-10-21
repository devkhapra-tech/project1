#include<bits/stdc++.h>
using namespace std;
vector<vector<pair<int,double>>> adj(vector<vector<double>>edges,int n){
    vector<vector<pair<int,double>>> adjlist(n);
    for(int i=0;i<edges.size();i++){
         int u=edges[i][0];
         int v=edges[i][1];
         double weight=edges[i][2];
         adjlist[u].push_back({v,weight});
         adjlist[v].push_back({u,weight});
    }
    return adjlist;
}
double dijkstra(vector<vector<pair<int,double>>> adjlist,int start,int goal){
    int n=adjlist.size();//no. of nodes
    vector<double> distance(n,INT_MAX);
    distance[start]=0.0;
    priority_queue<pair<double,int>,vector<pair<double,int>>,greater<pair<double,int>>> pq;
    pq.push({0.0,start});
    while(!pq.empty()){
        int node=pq.top().second ;
        double dist=pq.top().first;
        pq.pop();
        for(auto it:adjlist[node]){
            int adjnode=it.first;
            double adjdist=it.second;
            if(dist+adjdist<distance[adjnode]){
                distance[adjnode]=dist+adjdist;
                pq.push({distance[adjnode],adjnode});
            }
        }   
    }
    return distance[goal];
}
double heuristic(int u,int v,unordered_map<int,pair<double, double>> &station_pos){
    double x1=station_pos[u].first;
    double y1=station_pos[u].second;
    double x2=station_pos[v].first;
    double y2=station_pos[v].second;
    double dif1=(x2-x1)*(x2-x1);
    double dif2=(y2-y1)*(y2-y1);
    double ans=sqrt(dif1+dif2);
    return ans;
}
double astar(vector<vector<pair<int,double>>> adjlist,int start,int goal,unordered_map<int,pair<double, double>> &station_pos){
    priority_queue<pair<double,int>,vector<pair<double,int>>,greater<pair<double,int>>> pq; // fscore and node
    unordered_map<int,double> gscore;     //node and value   it is the distance of current node from the start node
    unordered_map<int,double> fscore;    //node and value   it is the distance of start node to the end node 
    for(int i=0;i<adjlist.size();i++){
        //INITIALISING ALL NODES VALUE TO NOT DEFINED 
        gscore[i]=INT_MAX;  
        fscore[i]=INT_MAX; 
    }
    gscore[start]=0;
    fscore[start]=0+heuristic(start,goal,station_pos);
    pq.push({fscore[start],start});
    while(!pq.empty()){
        int curr=pq.top().second;
        pq.pop();
        if(curr==goal){
            return gscore[curr];
        }
        for(auto it:adjlist[curr]){
            int adjnode=it.first;
            double weight=it.second;
            double tentative_gscore=gscore[curr]+weight;
            if(tentative_gscore<gscore[adjnode]){
                gscore[adjnode]=tentative_gscore;
                fscore[adjnode]=tentative_gscore+heuristic(adjnode,goal,station_pos);
                pq.push({fscore[adjnode],adjnode});
            }
        }
    }
    return -1.0;
}
int main(){

    vector<vector<double>> edges;
    edges.push_back({0,4,1.00});
    edges.push_back({0,80,0.80});
    edges.push_back({1,2,1.20});
    edges.push_back({1,84,1.20});
    edges.push_back({2,89,0.70});
    edges.push_back({3,84,1.10});
    edges.push_back({3,14,1.10});
    edges.push_back({4,79,1.70});
    edges.push_back({5,9,1.30});
    edges.push_back({5,15,0.90});
    edges.push_back({6,7,1.30});
    edges.push_back({6,79,1.30});
    edges.push_back({7,13,1.40});
    edges.push_back({8,79,1.80});
    edges.push_back({8,12,1.00});
    edges.push_back({9,75,1.40});
    edges.push_back({10,11,1.20});
    edges.push_back({10,77,0.80});
    edges.push_back({11,89,1.30});
    edges.push_back({11,34,2.30});
    edges.push_back({11,22,0.60});
    edges.push_back({13,77,0.60});
    edges.push_back({14,15,1.20});
    edges.push_back({16,18,1.30});
    edges.push_back({16,75,1.50});
    edges.push_back({17,18,1.20});
    edges.push_back({17,19,1.80});
    edges.push_back({19,20,0.90});
    edges.push_back({21,92,1.50});
    edges.push_back({21,87,1.80});
    edges.push_back({22,86,1.10});
    edges.push_back({23,24,1.30});
    edges.push_back({23,31,1.50});
    edges.push_back({24,40,1.20});
    edges.push_back({25,26,1.00});
    edges.push_back({25,78,1.80});
    edges.push_back({26,27,1.10});
    edges.push_back({27,28,1.00});
    edges.push_back({28,29,1.10});
    edges.push_back({29,30,0.90});
    edges.push_back({30,31,0.90});
    edges.push_back({32,92,1.80});
    edges.push_back({32,49,0.80});
    edges.push_back({33,82,0.90});
    edges.push_back({33,51,1.00});
    edges.push_back({34,45,1.10});
    edges.push_back({35,45,2.20});
    edges.push_back({35,44,1.10});
    edges.push_back({36,73,0.90});
    edges.push_back({36,54,1.60});
    edges.push_back({37,92,1.30});
    edges.push_back({37,41,1.10});
    edges.push_back({38,55,1.00});
    edges.push_back({38,87,1.20});
    edges.push_back({39,46,1.20});
    edges.push_back({39,85,0.90});
    edges.push_back({40,53,1.10});
    edges.push_back({41,43,1.00});
    edges.push_back({42,47,1.30});
    edges.push_back({42,44,0.90});
    edges.push_back({43,83,1.30});
    edges.push_back({46,91,1.00});
    edges.push_back({47,85,0.90});
    edges.push_back({48,50,0.90});
    edges.push_back({48,51,0.90});
    edges.push_back({49,86,0.90});
    edges.push_back({50,91,1.20});
    edges.push_back({52,53,1.00});
    edges.push_back({52,82,1.40});
    edges.push_back({55,56,1.20});
    edges.push_back({56,57,1.00});
    edges.push_back({57,58,1.20});
    edges.push_back({58,76,1.10});
    edges.push_back({59,60,0.80});
    edges.push_back({59,81,1.10});
    edges.push_back({60,61,2.80});
    edges.push_back({61,88,1.30});
    edges.push_back({62,64,1.70});
    edges.push_back({62,63,0.80});
    edges.push_back({63,65,1.00});
    edges.push_back({64,81,1.20});
    edges.push_back({65,84,1.20});
    edges.push_back({66,91,2.00});
    edges.push_back({66,90,2.00});
    edges.push_back({67,90,2.00});
    edges.push_back({67,88,2.00});
    edges.push_back({68,88,2.00});
    edges.push_back({68,75,2.00});
    edges.push_back({69,70,7.00});
    edges.push_back({69,71,3.50});
    edges.push_back({70,72,6.80});
    edges.push_back({71,78,3.10});
    edges.push_back({72,89,1.90});
    edges.push_back({74,94,0.90});
    edges.push_back({74,81,1.50});
    edges.push_back({74,93,2.00});
    edges.push_back({85,93,1.10});
    edges.push_back({90,94,2.00});
    
    
    vector<vector<pair<int,double>>> adjlist=adj(edges,95);
    unordered_map<string, int> station_num;

    // Inserting station names into the unordered_map with unique numbers
    station_num["AIIMS"] = 0;
    station_num["CHANDNI CHOWK"] = 1;
    station_num["CHAWRI BAZAR"] = 2;
    station_num["CIVIL LINES"] = 3;
    station_num["GREEN PARK"] = 4;
    station_num["GTB NAGAR"] = 5;
    station_num["JOR BAGH"] = 6;
    station_num["LOK KALYAN MARG"] = 7;
    station_num["MALVIYA NAGAR"] = 8;
    station_num["MODEL TOWN"] = 9;
    station_num["PATEL CHOWK"] = 10;
    station_num["RAJIV CHOWK"] = 11;
    station_num["SAKET"] = 12;
    station_num["UDYOG BHAWAN"] = 13;
    station_num["VIDHAN SABHA"] = 14;
    station_num["VISHWA VIDYALAYA"] = 15;
    station_num["ADARSH NAGAR"] = 16;
    station_num["HAIDERPUR"] = 17;
    station_num["JAHANGIRPURI"] = 18;
    station_num["ROHINI SECTOR 18"] = 19;
    station_num["SAMAYPUR BADLI"] = 20;
    station_num["AKSHARDHAM"] = 21;
    station_num["BARAKHAMBA ROAD"] = 22;
    station_num["DWARKA"] = 23;
    station_num["DWARKA MOR"] = 24;
    station_num["DWARKA SECTOR 8"] = 25;
    station_num["DWARKA SECTOR 9"] = 26;
    station_num["DWARKA SECTOR 10"] = 27;
    station_num["DWARKA SECTOR 11"] = 28;
    station_num["DWARKA SECTOR 12"] = 29;
    station_num["DWARKA SECTOR 13"] = 30;
    station_num["DWARKA SECTOR 14"] = 31;
    station_num["INDRAPRASTHA"] = 32;
    station_num["JANAKPURI EAST"] = 33;
    station_num["JHANDEWALAN"] = 34;
    station_num["KAROL BAGH"] = 35;
    station_num["KAUSHAMBI"] = 36;
    station_num["LAXMI NAGAR"] = 37;
    station_num["MAYUR VIHAR EXTENSION"] = 38;
    station_num["MOTI NAGAR"] = 39;
    station_num["NAWADA"] = 40;
    station_num["NIRMAN VIHAR"] = 41;
    station_num["PATEL NAGAR"] = 42;
    station_num["PREET VIHAR"] = 43;
    station_num["RAJENDRA PLACE"] = 44;
    station_num["RK ASHRAM"] = 45;
    station_num["RAMESH NAGAR"] = 46;
    station_num["SHADIPUR"] = 47;
    station_num["SUBHASH NAGAR"] = 48;
    station_num["SUPREME COURT"] = 49;
    station_num["TAGORE GARDEN"] = 50;
    station_num["TILAK NAGAR"] = 51;
    station_num["UTTAM NAGAR EAST"] = 52;
    station_num["UTTAM NAGAR WEST"] = 53;
    station_num["VAISHALI"] = 54;
    station_num["NEW ASHOK NAGAR"] = 55;
    station_num["NOIDA SECTOR 15"] = 56;
    station_num["NOIDA SECTOR 16"] = 57;
    station_num["NOIDA SECTOR 18"] = 58;
    station_num["KANHAIYA NAGAR"] = 59;
    station_num["KESHAV PURAM"] = 60;
    station_num["KOHAT ENCLAVE"] = 61;
    station_num["PRATAP NAGAR"] = 62;
    station_num["PUL BANGASH"] = 63;
    station_num["SHASTRI NAGAR"] = 64;
    station_num["TIS HAZARI"] = 65;
    station_num["ESI HOSPITAL"] = 66;
    station_num["SHAKURPUR"] = 67;
    station_num["SHALIMAR BAGH"] = 68;
    station_num["AEROCITY"] = 69;
    station_num["DHAULA KUAN"] = 70;
    station_num["IGI AIRPORT"] = 71;
    station_num["SHIVAJI STADIUM"] = 72;
    station_num["ANAND VIHAR ISBT"] = 73;
    station_num["ASHOK PARK MAIN"] = 74;
    station_num["AZADPUR"] = 75;
    station_num["BOTANICAL GARDEN"] = 76;
    station_num["CENTRAL SECRETARIAT"] = 77;
    station_num["DWARKA SECTOR 21"] = 78;
    station_num["HAUZ KHAS"] = 79;
    station_num["INA"] = 80;
    station_num["INDERLOK"] = 81;
    station_num["JANAKPURI WEST"] = 82;
    station_num["KARKARDUMA"] = 83;
    station_num["KASHMERE GATE"] = 84;
    station_num["KIRTI NAGAR"] = 85;
    station_num["MANDI HOUSE"] = 86;
    station_num["MAYUR VIHAR -I"] = 87;
    station_num["NETAJI SUBHASH PLACE"] = 88;
    station_num["NEW DELHI"] = 89;
    station_num["PUNJABI BAGH WEST"] = 90;
    station_num["RAJOURI GARDEN"] = 91;
    station_num["YAMUNA BANK"] = 92;
    station_num["SATGURU RAMSINGH MARG"] = 93;
    station_num["PUNJABI BAGH EAST"] = 94;
    unordered_map<int,pair<double, double>> station_pos;

    // Inserting data into the unordered_map
    station_pos[0] = make_pair(28.5668602, 77.2078058);
    station_pos[1] = make_pair(28.6605039, 77.2297797);
    station_pos[2] = make_pair(28.6501605, 77.2295008);
    station_pos[3] = make_pair(28.6768508, 77.2250299);
    station_pos[4] = make_pair(28.5585815, 77.2067183);
    station_pos[5] = make_pair(28.6981317, 77.2064113);
    station_pos[6] = make_pair(28.58823925, 77.21652774);
    station_pos[7] = make_pair(28.598385, 77.2029964);
    station_pos[8] = make_pair(28.5339201, 77.2124474);
    station_pos[9] = make_pair(28.7027136, 77.1939912);
    station_pos[10] = make_pair(28.6229664, 77.214031);
    station_pos[11] = make_pair(28.6327804, 77.2196996);
    station_pos[12] = make_pair(28.5244107, 77.2137253);
    station_pos[13] = make_pair(28.6105302, 77.2131076);
    station_pos[14] = make_pair(28.6863222, 77.2217271);
    station_pos[15] = make_pair(28.6950369, 77.2147192);
    station_pos[16] = make_pair(28.7144008, 77.1672884);
    station_pos[17] = make_pair(28.7301214, 77.1494029);
    station_pos[18] = make_pair(28.7259717, 77.162658);
    station_pos[19] = make_pair(28.7383477, 77.1398323);
    station_pos[20] = make_pair(28.7446158, 77.1382654);
    station_pos[21] = make_pair(28.61784195, 77.27948814);
    station_pos[22] =make_pair(28.6297676, 77.2250046);
    station_pos[23] = make_pair(28.5771915, 77.0442935);
    station_pos[24] = make_pair(28.61931, 77.0332792);
    station_pos[25] = make_pair(28.5656109, 77.0670366);
    station_pos[26] = make_pair(28.574272, 77.0653316);
    station_pos[27] = make_pair(28.5810577, 77.0574996);
    station_pos[28] = make_pair(28.5864469, 77.0494375);
    station_pos[29] =make_pair(28.5922362, 77.0407315);
    station_pos[30] = make_pair(28.597009, 77.0335407);
    station_pos[31] =make_pair(28.60225285, 77.02595656);
    station_pos[32] = make_pair(28.6206019, 77.249551);
    station_pos[33] = make_pair(28.633021, 77.086757);
    station_pos[34] = make_pair(28.6443188, 77.1999165);
    station_pos[35] = make_pair(28.6441866, 77.189132);
    station_pos[36] = make_pair(28.6453286, 77.3243485);
    station_pos[37] = make_pair(28.63055515, 77.27763317);
    station_pos[38] = make_pair(28.59415815, 77.29458935);
    station_pos[39] = make_pair(28.657859, 77.1424288);
    station_pos[40] = make_pair(28.6202758, 77.0450787);
    station_pos[41] = make_pair(28.63650605, 77.28683489);
    station_pos[42] = make_pair(28.6455858, 77.1686983);
    station_pos[43] = make_pair(28.6414952, 77.29537443);
    station_pos[44] = make_pair(28.6424955, 77.1782756);
    station_pos[45] = make_pair(28.6392436, 77.2085787);
    station_pos[46] = make_pair(28.6527444, 77.1316496);
    station_pos[47] = make_pair(28.6516362, 77.1582947);
    station_pos[48] = make_pair(28.6418074, 77.1087611);
    station_pos[49] = make_pair(28.6222879, 77.23943397);
    station_pos[50] = make_pair(28.6437639, 77.1128454);
    station_pos[51] = make_pair(28.6365482, 77.0964961);
    station_pos[52] = make_pair(28.6248465, 77.0652861);
    station_pos[53] = make_pair(28.621757, 77.0557122);
    station_pos[54] = make_pair(28.6440912, 77.06838558);
    station_pos[55] = make_pair(28.5890885, 77.3015085);
    station_pos[56] = make_pair(28.4808629, 77.0848883);
    station_pos[57] = make_pair(28.4808629, 77.0848883);
    station_pos[58] = make_pair(28.4808629, 77.0848883);
    station_pos[59] = make_pair(28.6800635, 77.1648445);
    station_pos[60] = make_pair(28.6889264, 77.1616833);
    station_pos[61] = make_pair(28.6980415, 77.1405393);
    station_pos[62] = make_pair(28.6664902, 77.19933568);
    station_pos[63] = make_pair(28.6663751, 77.20752912);
    station_pos[64] = make_pair(28.66997485, 77.18199977);
    station_pos[65] = make_pair(28.6671323, 77.21662796);
    station_pos[66] = make_pair(28.6580737, 77.1272678);
    station_pos[67] = make_pair(28.6857668, 77.1496094);
    station_pos[68] = make_pair(28.71745265, 77.15086655);
    station_pos[69] = make_pair(28.5487982, 77.1208089);
    station_pos[70] = make_pair(28.5918905, 77.1617034);
    station_pos[71] = make_pair(28.55489735, 77.08467458);
    station_pos[72] = make_pair(28.6315087, 77.2160589);
    station_pos[73] = make_pair(28.6467533, 77.3180037);
    station_pos[74] = make_pair(28.6716045, 77.155291);
    station_pos[75] = make_pair(28.7076568, 77.1755473);
    station_pos[76] = make_pair(28.5638964, 77.334332);
    station_pos[77] = make_pair(28.6158794, 77.2122822);
    station_pos[78] = make_pair(28.5518376, 77.0586489);
    station_pos[79] = make_pair(28.5442564, 77.2067072);
    station_pos[80] = make_pair(28.57440755, 77.21024148);
    station_pos[81] = make_pair(28.6517178, 77.2219388);
    station_pos[82] = make_pair(28.6289502, 77.0779235);
    station_pos[83] = make_pair(28.6491622, 77.30620811);
    station_pos[84] = make_pair(28.6517178, 77.2219388);
    station_pos[85] = make_pair(28.6532807, 77.1417729);
    station_pos[86] = make_pair(28.6255556, 77.234195);
    station_pos[87] = make_pair(28.603133, 77.2925892);
    station_pos[88] = make_pair(28.69605175, 77.15264042);
    station_pos[89] = make_pair(28.6436415, 77.2217373);
    station_pos[90] = make_pair(28.6703201, 77.1420875);
    station_pos[91] = make_pair(28.6421518, 77.11606038);
    station_pos[92] = make_pair(28.6232784, 77.26792405);
    station_pos[93] = make_pair(28.6619327, 77.1574772);
    station_pos[94] = make_pair(28.668945,77.1324614);

    
    string start;
    cout<<"input start state: "<<endl;
    getline(cin,start);
    string goal;
    cout<<"input goal state: "<<endl;
    getline(cin,goal);
    if(!station_num.count(start)||!station_num.count(goal)){
        cout<<" give valid input"<<endl;
    }
    int s=station_num[start];
    int g=station_num[goal];
    cout<<"by dijkstra: "<<dijkstra(adjlist,s,g)<<endl;
    cout<<" by astar : "<<astar(adjlist,s,g,station_pos)<<endl;

    return 0;
}