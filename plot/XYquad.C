#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>

void XYquad(){
  const int n = 33;	
  float t[n],roll[n], pitch[n], yaw[n], N[n],S[n],E[n],W[n];
  string text;
  ifstream data;

  string name = "sample_file";
  data.open((name+".txt").c_str());
  
  int k = 0;

  while(!data.eof() && k<n){
    t[k]=k;
    data>>text>>roll[k]>>pitch[k]>>yaw[k]>>N[k]>>S[k]>>E[k]>>W[k];
    k++;
  }
  
    TGraphErrors * grp_roll = new TGraph(n,t,roll);
    TGraphErrors * grp_pitch = new TGraph(n,t,pitch);
    TGraphErrors * grp_yaw = new TGraph(n,t,yaw);
    TGraphErrors * grp_N = new TGraph(n,t,N);
    TGraphErrors * grp_S = new TGraph(n,t,S);
    TGraphErrors * grp_E = new TGraph(n,t,E);
    TGraphErrors * grp_W = new TGraph(n,t,W);
    
    TCanvas * Ca0 = new TCanvas("Ca0","Canvas",1200,800);    

    Ca0->Divide(2,3);
    Ca0_1->cd();
    grp_roll->SetTitle("roll");  grp_roll->SetLineColor(kRed);
    grp_roll->SetMarkerStyle(20);  grp_roll->SetMarkerSize(1.0);
    grp_roll->SetMinimum(-180);  grp_roll->SetMaximum(180);
    grp_roll->GetXaxis()->SetTitle("t (A.U.)"); grp_roll->GetYaxis()->SetTitle("roll(degrees)");
    grp_roll->Draw("AP");

    Ca0_2->cd();
    grp_pitch->SetTitle("pitch");  grp_pitch->SetLineColor(kRed);
    grp_pitch->SetMarkerStyle(20);  grp_pitch->SetMarkerSize(1.0);
    grp_pitch->SetMinimum(-180);  grp_pitch->SetMaximum(180);
    grp_pitch->GetXaxis()->SetTitle("t (A.U.)"); grp_pitch->GetYaxis()->SetTitle("pitch(degrees)");
    grp_pitch->Draw("AP");
    
    Ca0_3->cd();
    grp_N->SetTitle("N");  grp_N->SetLineColor(kRed);
    grp_N->SetMarkerStyle(20);  grp_N->SetMarkerSize(1.0);
    grp_N->SetMinimum(-1.0);  grp_N->SetMaximum(1.0);
    grp_N->GetXaxis()->SetTitle("t (A.U.)"); grp_N->GetYaxis()->SetTitle("N(fraction)");
    grp_N->Draw("AP");

    Ca0_4->cd();
    grp_E->SetTitle("E");  grp_E->SetLineColor(kRed);
    grp_E->SetMarkerStyle(20);  grp_E->SetMarkerSize(1.0);
    grp_E->SetMinimum(-1.0);  grp_E->SetMaximum(1.0);
    grp_E->GetXaxis()->SetTitle("t (A.U.)"); grp_E->GetYaxis()->SetTitle("E(fraction)");
    grp_E->Draw("AP");

    Ca0_5->cd();
    grp_S->SetTitle("S");  grp_S->SetLineColor(kRed);
    grp_S->SetMarkerStyle(20);  grp_S->SetMarkerSize(1.0);
    grp_S->SetMinimum(-1.0);  grp_S->SetMaximum(1.0);
    grp_S->GetXaxis()->SetTitle("t (A.U.)"); grp_S->GetYaxis()->SetTitle("S(fraction)");
    grp_S->Draw("AP");

    Ca0_6->cd();
    grp_W->SetTitle("W");  grp_W->SetLineColor(kRed);
    grp_W->SetMarkerStyle(20);  grp_W->SetMarkerSize(1.0);
    grp_W->SetMinimum(-1.0);  grp_W->SetMaximum(1.0);
    grp_W->GetXaxis()->SetTitle("t (A.U.)"); grp_W->GetYaxis()->SetTitle("W(fraction)");
    grp_W->Draw("AP");

    Ca0->SaveAs("XYquad.png");
    exit (0);
            
}
