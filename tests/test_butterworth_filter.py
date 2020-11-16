import unittest
import numpy as np
from dg_tools.filter import ButterWorthFilter


class TestButterWorthFilter(unittest.TestCase):
    def test_init(self):
        my_filter = ButterWorthFilter('butterworth_filter')
        self.assertEqual(my_filter.name, 'butterworth_filter')

    def test_basic(self):

        my_filter = ButterWorthFilter('butterworth_filter')

        # Sample rate and desired cutoff frequencies (in Hz).
        order = 6
        fs = 5000.0
        sampling_time = 1./fs
        nyq_cutoff_perc = 0.24 # Keep 24% of the nyq frequency of the data

        my_filter.init(1, sampling_time, 0.1, 3)

        # Change the filter frequency to the desired value.
        my_filter.update(nyq_cutoff_perc, order)

        # Filter a noisy signal.
        T = 0.05
        nsamples = int(T * fs)
        t = np.linspace(0.0, T, nsamples, endpoint=False)
        a = 0.02
        f0 = 600.0
        x = 0.1 * np.sin(2 * np.pi * 1.2 * np.sqrt(t))
        x += 0.01 * np.cos(2 * np.pi * 312 * t + 0.1)
        x += a * np.cos(2 * np.pi * f0 * t + .11)
        x += 0.03 * np.cos(2 * np.pi * 2000 * t)

        y = []
        for i in range(x.size):
            my_filter.sin.value = np.array([x[i]])
            my_filter.sout.recompute(i)
            y.append(float(my_filter.sout.value))

        y = np.array(y)
        y_result = TestButterWorthFilter.y_result()

        np.testing.assert_array_almost_equal(y, y_result, decimal=5)

    @staticmethod
    def y_result():
        return np.array([
            0.05982900000000035,
            0.05978470395493348,
            0.05940012529707808,
            0.05783879723175209,
            0.05382866705116604,
            0.04637900986915262,
            0.035606941369028006,
            0.02310786641871588,
            0.011690026755536086,
            0.004505672084687912,
            0.0038045508269339487,
            0.009775859015817187,
            0.020111283350568953,
            0.030736789900242863,
            0.03756625981288171,
            0.038502494712118016,
            0.034601377341902204,
            0.029643461998719013,
            0.028192245813883476,
            0.03307456584609597,
            0.0436486656663503,
            0.055869042592986834,
            0.06422272809571736,
            0.06458734513207391,
            0.05649165060986188,
            0.04354550272668774,
            0.03174528605232425,
            0.02650457466192229,
            0.029970304719539497,
            0.04003412856106119,
            0.051570173148996735,
            0.05922322593766704,
            0.06024180131238762,
            0.05583866283808699,
            0.050346441341346415,
            0.048656809370532204,
            0.05335407575424839,
            0.06312711483025067,
            0.07335763048700374,
            0.07857784794074638,
            0.07548402484706504,
            0.0648610641307091,
            0.051352884178821674,
            0.041178828772336654,
            0.03898740233098087,
            0.04552702404399827,
            0.05736284131377259,
            0.06877383908909014,
            0.0748041094762302,
            0.07381251222728745,
            0.06816850141110717,
            0.06274138646872758,
            0.06203471129761282,
            0.06756007196733461,
            0.07689214470515297,
            0.08494533072302951,
            0.08678117265544832,
            0.08041590397071237,
            0.06809338589401427,
            0.055293623722605496,
            0.047989150125109625,
            0.04959723955941585,
            0.059249945320221944,
            0.07230801544151609,
            0.08282374966239298,
            0.08663470109084903,
            0.08342629177731616,
            0.07667340058061145,
            0.07153420127609697,
            0.07186188196706494,
            0.07799225517251758,
            0.08652053243459946,
            0.09219324409066078,
            0.09088961765102337,
            0.08204623602797735,
            0.06919070268016181,
            0.0582556407288333,
            0.05455132964663497,
            0.06001625959932419,
            0.07220658911580373,
            0.08557520351997094,
            0.09434921758783937,
            0.09546447312490045,
            0.09000277396055077,
            0.08237694441458708,
            0.07774916667263296,
            0.07910604150210701,
            0.08559381025116308,
            0.09302997665272313,
            0.09629678143694659,
            0.09230896603150211,
            0.08191297529991577,
            0.06965240819672637,
            0.06149968263381765,
            0.06174415317289741,
            0.07071348314585137,
            0.08455162165268214,
            0.09717917892112797,
            0.10340012588001281,
            0.10148963707792298,
            0.09390403681109835,
            0.08575738988914311,
            0.08191833991097452,
            0.08432764210238416,
            0.09098724648494394,
            0.09717169775192197,
            0.09818095353403487,
            0.09211208657137715,
            0.08112031247415655,
            0.07044115001954325,
            0.06568387171432595,
            0.06983865310369612,
            0.08161120399795184,
            0.09600124415892733,
            0.10682013248316641,
            0.1098212081628916,
            0.10477527129057351,
            0.09539924030192927,
            0.08721321256086395,
            0.08449717913636182,
            0.08801380519088947,
            0.09472040947556286,
            0.09960593681473245,
            0.09863555501741236,
            0.09116227386228712,
            0.08047328589284573,
            0.07214042870788956,
            0.07104754609406538,
            0.0787057325509586,
            0.09230763272544021,
            0.10606165358797275,
            0.11410656817114112,
            0.11345943422353962,
            0.10544051854835973,
            0.0948221566618453,
            0.08719249051619565,
            0.08596583759683818,
            0.09065297962631581,
            0.09731747662240037,
            0.1009318351875239,
            0.09833159528334363,
            0.0901265355687857,
            0.08050199181692724,
            0.07500485099288366,
            0.07749624239097455,
            0.08793791201646348,
            0.10222573888694378,
            0.11418775915760829,
            0.11870492170874043,
            0.11428724902907489,
            0.10374811836653401,
            0.0926266323751482,
            0.08621994326070564,
            0.08683577203120853,
            0.0927258173498433,
            0.09925924758486807,
            0.10166113817266842,
            0.09779552119961601,
            0.08946177006002344,
            0.08147075777416143,
            0.07900276018283285,
            0.08467973143234077,
            0.0969558294356989,
            0.11073405152134819,
            0.1198985041773955,
            0.1204336692757842,
            0.11246479986100635,
            0.100132810168895,
            0.08938702229931812,
            0.08487675364696541,
            0.08761803088422926,
            0.09466708647797306,
            0.10094229871660435,
            0.10217949979041521,
            0.09737841284734765,
            0.08940262087494398,
            0.08339432189376318,
            0.08386688886381316,
            0.09207632648152066,
            0.10511426598619446,
            0.11725748633064768,
            0.12287212850659612,
            0.11932711222025698,
            0.10836567869016386,
            0.09519008071285268,
            0.08576009372836768,
            0.08374699093454416,
            0.08876481673046867,
            0.09681098580608219,
            0.10263244457903362,
            0.1027139751480972,
            0.09724266619032451,
            0.08997344034499997,
            0.0860804067975417,
            0.08916611265472124,
            0.09908711378954363,
            0.11180341171505676,
            0.12136672560369813,
            0.12300730548246024,
            0.11565710715126924,
            0.1025600696168176,
            0.08962915636157093,
            0.0824219330911139,
            0.08335354887034635,
            0.09061382966192862,
            0.09935015487669538,
            0.10443954614357304,
            0.10332452668444857,
            0.09737347208236506,
            0.09102219114446944,
            0.08918876859831701,
            0.09438563998245718,
            0.10512673179927826,
            0.11653501168022214,
            0.12284136220851549,
            0.1204512176938175,
            0.10992107870691786,
            0.09576795016888229,
            0.08420500036541598,
            0.0799931068795788,
            0.08409066231159978,
            0.09333584717872322,
            0.10230166925678155,
            0.10630365913704899,
            0.10391005492509438,
            0.09760723303724833,
            0.09227170922191416,
            0.09230302958256072,
            0.09901252111938522,
            0.10970791741212585,
            0.11901009903710949,
            0.1217050705165868,
            0.11559361918476792,
            0.10279729033296048,
            0.08878567541536003,
            0.07963438360244365,
            0.07896145716678789,
            0.0861667172853191,
            0.09690485665552473,
            0.10550360141645154,
            0.10801563483199039,
            0.1042486446175021,
            0.09768842753976643,
            0.09339108563709111,
            0.09501018828265607,
            0.10261460042363876,
            0.11250570569757029,
            0.11915388229039853,
            0.11822177761376101,
            0.10902183599935351])
