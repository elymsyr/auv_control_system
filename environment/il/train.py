import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import pandas as pd
import numpy as np
import h5py
from sklearn.model_selection import train_test_split

# Custom Dataset
class AUV_Dataset(Dataset):
    def __init__(self, csv_file):
        data = pd.read_csv(csv_file)
        with h5py.File('mpc_data.h5', 'r') as hf:
            # Stack x_current and x_desired to form input features
            self.X = np.hstack((hf['x_current'][:], hf['x_desired'][:]))
            self.y = hf['u_opt'][:]

    def __len__(self):
        return len(self.X)
    
    def __getitem__(self, idx):
        # Directly access the numerical arrays
        x = self.X[idx]
        y = self.y[idx]
        return torch.FloatTensor(x), torch.FloatTensor(y)

# Neural Network Architecture (unchanged)
class FossenNet(nn.Module):
    def __init__(self):
        super(FossenNet, self).__init__()
        self.layers = nn.Sequential(
            nn.Linear(24, 256),    # 12 current + 12 desired states
            nn.BatchNorm1d(256),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(256, 128),
            nn.BatchNorm1d(128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 6)       # 6 thruster forces
        )
        
    def forward(self, x):
        return self.layers(x)

# Training Configuration (unchanged)
config = {
    'batch_size': 32,
    'lr': 1e-3,
    'epochs': 200,
    'weight_decay': 1e-5
}

def train():
    # Load Data
    dataset = AUV_Dataset('mpc_data.csv')
    train_data, val_data = train_test_split(dataset, test_size=0.2)
    
    # Initialize model, criterion, optimizer (unchanged)
    model = FossenNet()
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), 
                          lr=config['lr'], 
                          weight_decay=config['weight_decay'])
    
    # Data Loaders (unchanged)
    train_loader = DataLoader(train_data, batch_size=config['batch_size'], shuffle=True)
    val_loader = DataLoader(val_data, batch_size=config['batch_size'])
    
    best_loss = float('inf')
    
    for epoch in range(config['epochs']):
        # Training loop (unchanged)
        model.train()
        train_loss = 0
        for inputs, targets in train_loader:
            optimizer.zero_grad()
            outputs = model(inputs)
            loss = criterion(outputs, targets)
            loss.backward()
            optimizer.step()
            train_loss += loss.item()
        
        # Validation loop (unchanged)
        model.eval()
        val_loss = 0
        with torch.no_grad():
            for inputs, targets in val_loader:
                outputs = model(inputs)
                val_loss += criterion(outputs, targets).item()
                
        # Save best model (unchanged)
        avg_val_loss = val_loss/len(val_loader)
        if avg_val_loss < best_loss:
            best_loss = avg_val_loss
            torch.save(model.state_dict(), 'fossen_net.pth')
            
        print(f'Epoch {epoch+1}/{config["epochs"]} | '
              f'Train Loss: {train_loss/len(train_loader):.4f} | '
              f'Val Loss: {avg_val_loss:.4f}')

if __name__ == '__main__':
    train()