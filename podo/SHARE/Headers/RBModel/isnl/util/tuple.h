#ifndef _MYLIB_TUPLE_
#define _MYLIB_TUPLE_

	template<class T1, class T2>
	class tuple2{
	public:
		T1 e1; T2 e2;
		tuple2()                                                                                                                 :e1(),e2(){}
		tuple2(const T1& e1, const T2& e2)                                                                                       :e1(e1),e2(e2){}
	};
	template<class T1, class T2, class T3>
	class tuple3{
	public:
		T1 e1; T2 e2; T3 e3;
		tuple3()                                                                                                                 :e1(),e2(),e3(){}
		tuple3(const T1& e1, const T2& e2, const T3& e3)                                                                         :e1(e1),e2(e2),e3(e3){}
	};
	template<class T1, class T2, class T3, class T4>
	class tuple4{
	public:
		T1 e1; T2 e2; T3 e3; T4 e4;
		tuple4()                                                                                                                :e1(),e2(),e3(),e4(){}
		tuple4(const T1& e1, const T2& e2, const T3& e3, const T4& e4)                                                          :e1(e1),e2(e2),e3(e3),e4(e4){}
	};
	template<class T1, class T2, class T3, class T4, class T5>
	class tuple5{
	public:
		T1 e1; T2 e2; T3 e3; T4 e4; T5 e5;
		tuple5()                                                                                                                :e1(),e2(),e3(),e4(),e5(){}
		tuple5(const T1& e1, const T2& e2, const T3& e3, const T4& e4, const T5& e5)                                            :e1(e1),e2(e2),e3(e3),e4(e4),e5(e5){}
	};
	template<class T1, class T2, class T3, class T4, class T5, class T6>
	class tuple6{
	public:
		T1 e1; T2 e2; T3 e3; T4 e4; T5 e5; T6 e6;
		tuple6()                                                                                                                :e1(),e2(),e3(),e4(),e5(),e6(){}
		tuple6(const T1& e1, const T2& e2, const T3& e3, const T4& e4, const T5& e5, const T6& e6)                              :e1(e1),e2(e2),e3(e3),e4(e4),e5(e5),e6(e6){}
	};
	template<class T1, class T2, class T3, class T4, class T5, class T6, class T7>
	class tuple7{
	public:
		T1 e1; T2 e2; T3 e3; T4 e4; T5 e5; T6 e6; T7 e7;
		tuple7()                                                                                                               :e1(),e2(),e3(),e4(),e5(),e6(),e7(){}
		tuple7(const T1& e1, const T2& e2, const T3& e3, const T4& e4, const T5& e5, const T6& e6, const T7& e7)               :e1(e1),e2(e2),e3(e3),e4(e4),e5(e5),e6(e6),e7(e7){}
	};
	template<class T1, class T2, class T3, class T4, class T5, class T6, class T7, class T8>
	class tuple8{
	public:
		T1 e1; T2 e2; T3 e3; T4 e4; T5 e5; T6 e6; T7 e7; T8 e8;
		tuple8()                                                                                                               :e1(),e2(),e3(),e4(),e5(),e6(),e7(),e8(){}
		tuple8(const T1& e1, const T2& e2, const T3& e3, const T4& e4, const T5& e5, const T6& e6, const T7& e7, const T8& e8) :e1(e1),e2(e2),e3(e3),e4(e4),e5(e5),e6(e6),e7(e7),e8(e8){}
	};
	template<class T1, class T2>
	tuple2<T1,T2>                   tuple(const T1& e1, const T2& e2)                                                                                     { return tuple2(e1,e2); }
	template<class T1, class T2, class T3>
	tuple3<T1,T2,T3>                tuple(const T1& e1, const T2& e2, const T3& e3)                                                                       { return tuple3(e1,e2,e3); }
	template<class T1, class T2, class T3, class T4>
	tuple4<T1,T2,T3,T4>             tuple(const T1& e1, const T2& e2, const T3& e3, const T4& e4)                                                         { return tuple4(e1,e2,e3,e4); }
	template<class T1, class T2, class T3, class T4, class T5>
	tuple5<T1,T2,T3,T4,T5>          tuple(const T1& e1, const T2& e2, const T3& e3, const T4& e4, const T5& e5)                                           { return tuple5(e1,e2,e3,e4,e5); }
	template<class T1, class T2, class T3, class T4, class T5, class T6>
	tuple6<T1,T2,T3,T4,T5,T6>       tuple(const T1& e1, const T2& e2, const T3& e3, const T4& e4, const T5& e5, const T6& e6)                             { return tuple6(e1,e2,e3,e4,e5,e6); }
	template<class T1, class T2, class T3, class T4, class T5, class T6, class T7>
	tuple7<T1,T2,T3,T4,T5,T6,T7>    tuple(const T1& e1, const T2& e2, const T3& e3, const T4& e4, const T5& e5, const T6& e6, const T7& e7)               { return tuple7(e1,e2,e3,e4,e5,e6,e7); }
	template<class T1, class T2, class T3, class T4, class T5, class T6, class T7, class T8>
	tuple8<T1,T2,T3,T4,T5,T6,T7,T8> tuple(const T1& e1, const T2& e2, const T3& e3, const T4& e4, const T5& e5, const T6& e6, const T7& e7, const T8& e8) { return tuple8(e1,e2,e3,e4,e5,e6,e7,e8); }

#endif