module top (clk, a, b, c, d, s, out1, out2, out3);
  input clk, a, b, c, d, s;
  output out1, out2, out3;
  wire n1, n2, n3, n4, n5, q, qn;

  NAND2_X1 u_nand (.A1(a), .A2(b), .ZN(n1));
  AOI21_X1 u_aoi (.A(n1), .B1(c), .B2(d), .ZN(n2));
  XOR2_X1 u_xor (.A(n2), .B(n1), .Z(n3));
  MUX2_X1 u_mux (.A(n3), .B(n2), .S(s), .Z(n4));
  INV_X1 u_inv (.A(n4), .ZN(n5));
  DFF_X1 u_ff (.D(n5), .CK(clk), .Q(q), .QN(qn));
  BUF_X2 u_buf (.A(q), .Z(out1));
  NOR2_X1 u_nor (.A1(qn), .A2(n2), .ZN(out2));
  OAI22_X1 u_oai (.A1(q), .A2(n1), .B1(n3), .B2(d), .ZN(out3));
endmodule
