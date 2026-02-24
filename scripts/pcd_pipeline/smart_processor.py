"""
Smart Grid Processor
Binaları koruyarak sadece küçük delikleri doldurur ve inflation uygular
"""
import numpy as np
from scipy.ndimage import binary_dilation, binary_erosion, label

class SmartGridProcessor:
    def __init__(self, inflation_radius_meters=0.5, fill_small_holes_only=True):
        self.inflation_radius_meters = inflation_radius_meters
        self.fill_small_holes = fill_small_holes_only
    
    def process(self, occupancy_grid, grid_resolution):
        """
        Akıllı grid işleme:
        1. Küçük delikleri doldur (binaların içindeki boşluklar)
        2. Binary inflation (güvenlik marjini)
        
        Büyük açık alanları ve binaları korur!
        """
        processed = occupancy_grid.copy()
        
        # 1. Sadece küçük delikleri doldur
        if self.fill_small_holes:
            processed = self._fill_small_holes_only(processed, max_hole_size=100)
        
        # 2. Binary inflation
        inflation_cells = int(self.inflation_radius_meters / grid_resolution)
        if inflation_cells > 0:
            structure = self._create_circular_structure(inflation_cells)
            processed = binary_dilation(processed, structure=structure).astype(np.uint8)
        
        return processed
    
    def _fill_small_holes_only(self, grid, max_hole_size=100):
        """
        Sadece küçük delikleri doldur, büyük açık alanları koru
        
        Args:
            max_hole_size: Bu hücre sayısından küçük delikler doldurulur
        """
        # Ters grid (0=engel, 1=boş)
        inverse = 1 - grid
        
        # Bağlı bileşenleri bul
        labeled, num_features = label(inverse)
        
        # Her bileşenin boyutunu hesapla
        filled = grid.copy()
        for region_id in range(1, num_features + 1):
            region_mask = (labeled == region_id)
            region_size = np.sum(region_mask)
            
            # Küçük bileşenleri (delikler) doldur
            if region_size < max_hole_size:
                filled[region_mask] = 1
        
        return filled
    
    def _create_circular_structure(self, radius):
        """Circular structuring element"""
        size = 2 * radius + 1
        center = radius
        y, x = np.ogrid[:size, :size]
        mask = (x - center)**2 + (y - center)**2 <= radius**2
        return mask
