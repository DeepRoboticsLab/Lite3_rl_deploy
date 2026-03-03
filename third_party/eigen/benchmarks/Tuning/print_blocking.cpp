#include <Eigen/Core>
#include <cstdio>

using namespace Eigen;
using namespace Eigen::internal;

int main() {
  printf("%-8s %-8s %-8s | %-8s %-8s %-8s | %-8s %-8s %-8s | %-8s %-8s %-8s\n", "m", "n", "k", "kc_f", "mc_f", "nc_f",
         "kc_d", "mc_d", "nc_d", "mr_f", "nr_f", "LhsPr_f");

  // Print gebp_traits info
  {
    using Traits = gebp_traits<float, float>;
    printf("Float traits: mr=%d, nr=%d, LhsProgress=%d, RhsProgress=%d, NumberOfRegisters=%d\n", Traits::mr, Traits::nr,
           Traits::LhsProgress, Traits::RhsProgress, Traits::NumberOfRegisters);
  }
  {
    using Traits = gebp_traits<double, double>;
    printf("Double traits: mr=%d, nr=%d, LhsProgress=%d, RhsProgress=%d, NumberOfRegisters=%d\n", Traits::mr,
           Traits::nr, Traits::LhsProgress, Traits::RhsProgress, Traits::NumberOfRegisters);
  }

  // Print cache sizes
  std::ptrdiff_t l1, l2, l3;
  manage_caching_sizes(GetAction, &l1, &l2, &l3);
  printf("Cache sizes: L1=%ld, L2=%ld, L3=%ld\n", (long)l1, (long)l2, (long)l3);

  for (int size : {8, 16, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 384, 448, 512, 768, 1024, 1536, 2048}) {
    {
      Index kf = size, mf = size, nf = size;
      computeProductBlockingSizes<float, float>(kf, mf, nf);
      Index kd = size, md = size, nd = size;
      computeProductBlockingSizes<double, double>(kd, md, nd);
      printf("%-8d %-8d %-8d | %-8ld %-8ld %-8ld | %-8ld %-8ld %-8ld\n", size, size, size, (long)kf, (long)mf, (long)nf,
             (long)kd, (long)md, (long)nd);
    }
  }

  // Non-square
  for (auto [m, n, k] :
       std::initializer_list<std::tuple<int, int, int>>{{64, 64, 1024}, {1024, 64, 64}, {64, 1024, 64}}) {
    Index kf = k, mf = m, nf = n;
    computeProductBlockingSizes<float, float>(kf, mf, nf);
    Index kd = k, md = m, nd = n;
    computeProductBlockingSizes<double, double>(kd, md, nd);
    printf("%-8d %-8d %-8d | %-8ld %-8ld %-8ld | %-8ld %-8ld %-8ld\n", m, n, k, (long)kf, (long)mf, (long)nf, (long)kd,
           (long)md, (long)nd);
  }

  return 0;
}
